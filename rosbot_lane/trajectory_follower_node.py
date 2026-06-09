#!/usr/bin/env python3
"""
Trajectory follower with segment-based approach.

Handles multiple direction flips:
- Detects all flip points at load time
- Drives to each flip point as a goal
- Switches direction and continues to next flip point
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import math
from enum import Enum

# Add package path for imports
import sys
import os
package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if package_path not in sys.path:
    sys.path.insert(0, package_path)

from rosbot_lane.core.pure_pursuit import PurePursuitController, PurePursuitConfig
from rosbot_lane.core.trajectory import Trajectory


class State(Enum):
    WAITING_FOR_LOCALIZATION = 0
    GO_TO_START = 1
    ALIGN_AT_START = 2
    FOLLOWING_SEGMENT = 3  # Follow current segment toward its goal (flip point)
    SEGMENT_TRANSITION = 4  # Brief pause/align between segments
    ALIGN_AT_END = 5
    COMPLETE = 6


class TrajectoryFollowerNode(Node):

    def __init__(self):
        super().__init__('trajectory_follower_node')
        
        # ===== PARAMETERS =====
        trajectory_file = '/home/sharjeel-ahmad/Documents/rosbot_ws/src/rosbot_lane/config/slam_trajectory.csv'
        
        # Pure Pursuit config
        pp_config = PurePursuitConfig(
            lookahead_distance=0.3,  # Shorter lookahead for tighter following
            min_lookahead=0.15,
            max_lookahead=0.5,
            max_speed=0.15,
            min_speed=0.03,
            curve_speed=0.08,
            reverse_speed=0.10,
            max_angular=1.0,
            kp_angular=1.5,
            rotate_first_threshold=0.3,
            goal_tolerance=0.01,
            angle_tolerance=0.05,
        )
        
        # Segment transition tolerance
        self._segment_goal_tolerance = 0.15
        
        # LIDAR config (LIDAR is backwards - front at ±π)
        self._front_angle_range = math.radians(30)
        self._stop_distance = 0.35
        
        # =====================
        
        # Load trajectory
        self._trajectory = Trajectory(trajectory_file)
        self.get_logger().info(f'Loaded {self._trajectory.num_waypoints} waypoints in {self._trajectory.num_segments} segments')
        
        # Controller
        self._controller = PurePursuitController(pp_config)
        
        # State
        self._state = State.WAITING_FOR_LOCALIZATION
        
        # Pose
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        
        # LIDAR
        self._front_dist = float('inf')
        
        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        # Subscribers
        self.create_subscription(LaserScan, '/rosbot3/scan_filtered', self._scan_cb, 10)
        
        # Publisher
        self._cmd_pub = self.create_publisher(TwistStamped, '/rosbot3/cmd_vel', 10)
        
        # Control loop at 20Hz
        self.create_timer(0.05, self._control_loop)
        
        self.get_logger().info('Trajectory follower ready.')

    def _scan_cb(self, msg: LaserScan):
        """Process LIDAR scan - extract front distance."""
        front_dists = []
        
        for i, dist in enumerate(msg.ranges):
            if dist < msg.range_min or dist > msg.range_max:
                continue
            if math.isnan(dist) or math.isinf(dist):
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            
            # LIDAR backwards - front at ±π
            if abs(abs(angle) - math.pi) <= self._front_angle_range:
                front_dists.append(dist)
        
        self._front_dist = min(front_dists) if front_dists else float('inf')

    def _update_pose(self) -> bool:
        """Get current pose from TF."""
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return False
        
        self._x = t.transform.translation.x
        self._y = t.transform.translation.y
        
        q = t.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._theta = math.atan2(siny, cosy)
        
        return True

    def _control_loop(self):
        """Main control loop."""
        if not self._update_pose():
            if self._state == State.WAITING_FOR_LOCALIZATION:
                return
            self.get_logger().warn('Lost localization!', throttle_duration_sec=2.0)
            return
        
        if self._state == State.WAITING_FOR_LOCALIZATION:
            self._state = State.GO_TO_START
            self.get_logger().info(f'Localized at ({self._x:.2f}, {self._y:.2f}, θ={math.degrees(self._theta):.1f}°)')
            self.get_logger().info(f'Start: ({self._trajectory.start.x:.2f}, {self._trajectory.start.y:.2f})')
            return
        
        # State machine
        if self._state == State.GO_TO_START:
            self._handle_go_to_start()
        elif self._state == State.ALIGN_AT_START:
            self._handle_align_at_start()
        elif self._state == State.FOLLOWING_SEGMENT:
            self._handle_following_segment()
        elif self._state == State.SEGMENT_TRANSITION:
            self._handle_segment_transition()
        elif self._state == State.ALIGN_AT_END:
            self._handle_align_at_end()

    def _handle_go_to_start(self):
        """Navigate to trajectory start."""
        start = self._trajectory.start
        
        linear, angular, distance, heading_error, reached = self._controller.compute_control(
            self._x, self._y, self._theta,
            start.x, start.y,
            is_reverse=False,
            speed_factor=1.0
        )
        
        if reached:
            self._stop()
            self._state = State.ALIGN_AT_START
            self.get_logger().info('Reached start, aligning...')
            return
        
        self._publish_cmd(linear, angular)
        
        self.get_logger().info(
            f'[GO_TO_START] Dist: {distance:.2f}m | Err: {math.degrees(heading_error):.1f}° | Spd: {linear:.2f}',
            throttle_duration_sec=0.5
        )

    def _handle_align_at_start(self):
        """Align heading at start."""
        start = self._trajectory.start
        
        angular, aligned = self._controller.compute_align_control(self._theta, start.theta)
        
        if aligned:
            self._stop()
            self._trajectory.reset()
            self._state = State.FOLLOWING_SEGMENT
            seg = self._trajectory.current_segment
            self.get_logger().info(f'Aligned! Starting {seg}')
            return
        
        self._publish_cmd(0.0, angular)

    def _handle_following_segment(self):
        """Follow current segment toward its goal (the flip point)."""
        seg = self._trajectory.current_segment
        
        if not seg:
            self._stop()
            self._state = State.ALIGN_AT_END
            self.get_logger().info('No more segments!')
            return
        
        # Check if we reached the segment goal
        if self._trajectory.reached_segment_goal(self._x, self._y, self._segment_goal_tolerance):
            self._stop()
            
            # Move to next segment
            self._trajectory.advance_segment()
            
            if self._trajectory.is_complete:
                self._state = State.ALIGN_AT_END
                self.get_logger().info('All segments complete!')
            else:
                self._state = State.SEGMENT_TRANSITION
                new_seg = self._trajectory.current_segment
                self.get_logger().info(f'Reached goal! Transitioning to {new_seg}')
            return
        
        # Advance past waypoints we've passed
        
        
        # Get lookahead waypoint within segment
        la_idx, la_wp = self._trajectory.find_lookahead_in_segment(
            self._x, self._y,
            num_points_ahead=10
        )
        
        # Distance to lookahead
        dist_to_la = math.sqrt((la_wp.x - self._x)**2 + (la_wp.y - self._y)**2)
        
        # Is this segment reverse?
        is_reverse = seg.is_reverse
        
        # Obstacle check (only for forward)
        if not is_reverse and self._front_dist < self._stop_distance:
            self._stop()
            self.get_logger().info(f'[STOP] Obstacle at {self._front_dist:.2f}m', throttle_duration_sec=0.5)
            return
        
        # Compute control
        linear, angular, distance, heading_error, _ = self._controller.compute_control(
            self._x, self._y, self._theta,
            la_wp.x, la_wp.y,
            is_reverse=is_reverse,
            speed_factor=1.0
        )
        
        self._publish_cmd(linear, angular)
        
        # Logging
        mode = "REV" if is_reverse else "FWD"
        goal_dist = self._trajectory.distance_to_segment_goal(self._x, self._y)
        self.get_logger().info(
            f'[{mode}] Seg {self._trajectory.current_segment_idx+1}/{self._trajectory.num_segments} | '
            f'WP {self._trajectory.current_wp_idx} → LA:{la_idx} | '
            f'Goal: {goal_dist:.2f}m | Spd: {linear:.2f}',
            throttle_duration_sec=0.5
        )

    def _handle_segment_transition(self):
        """Brief transition between segments - just continue to next segment."""
        # Could add alignment here if needed
        # For now, just immediately continue
        self._state = State.FOLLOWING_SEGMENT
        seg = self._trajectory.current_segment
        if seg:
            self.get_logger().info(f'Starting {seg}')

    def _handle_align_at_end(self):
        """Align heading at end."""
        end = self._trajectory.end
        
        angular, aligned = self._controller.compute_align_control(self._theta, end.theta)
        
        if aligned:
            self._stop()
            self._state = State.COMPLETE
            self.get_logger().info('=== TRAJECTORY COMPLETE ===')
            return
        
        self._publish_cmd(0.0, angular)

    def _publish_cmd(self, linear: float, angular: float):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = linear
        twist.twist.angular.z = angular
        self._cmd_pub.publish(twist)

    def _stop(self):
        self._publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop()
        node.get_logger().info('Stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()