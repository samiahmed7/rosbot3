#!/usr/bin/env python3
"""Lane keeping node - uses core modules."""

import rclpy
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped

from rosbot_lane.core.config import load_config
from rosbot_lane.core.segmentation import segment_white
from rosbot_lane.core.detection import fit_lane_lines, apply_corridor_mask, eval_poly_at_y
from rosbot_lane.core.geometry import compute_lane_error, compute_lane_error_rl_only, compute_lane_error_cl_only
from rosbot_lane.core.control import LaneController
from rosbot_lane.core.logging import LaneLogger
from rosbot_lane.core.debug import draw_debug_frame


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = WORKSPACE_ROOT / 'config' / 'lane_params.yaml'
LOG_PATH = WORKSPACE_ROOT / 'lane_log.csv'


class LaneKeepingNode(Node):

    def __init__(self):
        super().__init__('lane_keeping_node')
        self.get_logger().info('Lane keeping node starting...')

        self.cfg = load_config(CONFIG_PATH)
        self.bridge = CvBridge()
        self._frame_num = 0
        self._no_detection_count = 0
        self._smoothed_error = 0.0

        # Corridor reference lines
        self._mask_cl_line = None
        self._mask_rl_line = None

        # Core modules
        self.controller = LaneController(self.cfg.kp, self.cfg.kd)
        self.logger = LaneLogger(str(LOG_PATH))

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.create_subscription(Image, self.cfg.topic_image, self._image_cb, sensor_qos)
        self._cmd_pub = self.create_publisher(TwistStamped, self.cfg.topic_cmd_vel, 10)

        self._save_debug_dir = WORKSPACE_ROOT / self.cfg.save_debug_dir
        if self.cfg.save_debug_frames:
            self._save_debug_dir.mkdir(parents=True, exist_ok=True)
            # Clear frames from the previous run so each run's capture set
            # is unambiguous — frame numbers restart at 1 every run, so
            # stale files would otherwise sit alongside and mix with new
            # ones under colliding names.
            for f in self._save_debug_dir.glob('*.jpg'):
                f.unlink()

    def _image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        self._frame_num += 1
        strip_y = int(self.cfg.img_height * self.cfg.strip_top_fraction)
        strip = frame[strip_y:, :]
        sh = strip.shape[0]

        # Segmentation
        binary = segment_white(
            strip, self.cfg.hsv_low, self.cfg.hsv_high,
            self.cfg.morph_close_iterations
        )

        # Corridor mask
        binary = apply_corridor_mask(
            binary, self._mask_cl_line, self._mask_rl_line,
            self.cfg.corridor_px
        )

        # Detection
        cl_line, rl_line = fit_lane_lines(binary, self.cfg.img_width)

        # Compute 3 points per line
        cl_top_x = eval_poly_at_y(cl_line, 0) if cl_line is not None else None
        cl_mid_x = eval_poly_at_y(cl_line, sh / 2) if cl_line is not None else None
        cl_bot_x = eval_poly_at_y(cl_line, sh) if cl_line is not None else None
        rl_top_x = eval_poly_at_y(rl_line, 0) if rl_line is not None else None
        rl_mid_x = eval_poly_at_y(rl_line, sh / 2) if rl_line is not None else None
        rl_bot_x = eval_poly_at_y(rl_line, sh) if rl_line is not None else None

        # Update corridor references — clear on failure too, so a lost line
        # doesn't leave the search corridor permanently stuck around its
        # last-known (and increasingly wrong) position as the robot moves.
        self._mask_cl_line = cl_line if cl_top_x is not None else None
        self._mask_rl_line = rl_line if rl_top_x is not None else None

        # Compute error. CL/RL are assigned by which side of frame-center a
        # detected line falls on, not by whether it's physically the dashed
        # or solid line — during a sharp turn the robot's heading can rotate
        # enough that the solid line swings to the "CL" side and the dashed
        # line ends up "RL". So neither slot can be trusted as the reliably-
        # detected one: fall back to whichever line is actually present,
        # and only treat it as truly lost when both are gone at once.
        if cl_bot_x is not None and rl_bot_x is not None:
            error, lane_centre_x = compute_lane_error(
                cl_bot_x, rl_bot_x, self.cfg.img_width, self.cfg.lane_width_px
            )
        elif rl_bot_x is not None:
            error, lane_centre_x = compute_lane_error_rl_only(rl_bot_x, self.cfg.img_width)
        elif cl_bot_x is not None:
            error, lane_centre_x = compute_lane_error_cl_only(cl_bot_x, self.cfg.img_width)
        else:
            error, lane_centre_x = 0.0, self.cfg.img_width / 2.0

        # Control
        if cl_bot_x is None and rl_bot_x is None:
            self._no_detection_count += 1
            if self._no_detection_count >= self.cfg.no_detection_stop_frames:
                self._stop()
                self.get_logger().warn('No lane lines detected', throttle_duration_sec=1.0)
        else:
            self._no_detection_count = 0
            # Smooth the error (EMA) before it reaches the controller. A
            # single noisy frame (e.g. a fit pulled toward wall clutter that
            # isn't extreme enough to be rejected outright) shouldn't be
            # able to produce a full-strength commit now that kp is high
            # enough to actually act on it — a real, sustained curve still
            # comes through fine since it stays large across many frames.
            smoothing_alpha = 0.4
            self._smoothed_error = (
                smoothing_alpha * error + (1 - smoothing_alpha) * self._smoothed_error
            )
            self._drive(self._smoothed_error)

        # Logging
        self.logger.log(
            self._frame_num,
            cl_top_x, cl_mid_x, cl_bot_x,
            rl_top_x, rl_mid_x, rl_bot_x,
            lane_centre_x, error
        )

        # Debug — build the view if we need to show it and/or save it.
        # Saving is unconditional on every no-detection frame (the
        # interesting failure moments) plus a periodic cadence otherwise,
        # via cv2.imwrite — works headless, independent of whatever GUI
        # backend cv2.imshow does or doesn't have.
        no_lines = cl_bot_x is None and rl_bot_x is None
        should_save = self.cfg.save_debug_frames and (
            no_lines or self._frame_num % self.cfg.save_debug_every_n == 0
        )
        if self.cfg.show_debug or should_save:
            vis, bottom_row = draw_debug_frame(
                frame, strip, binary, cl_line, rl_line,
                cl_top_x, cl_mid_x, cl_bot_x,
                rl_top_x, rl_mid_x, rl_bot_x,
                lane_centre_x, error, strip_y, sh,
                self.cfg.img_width, self.cfg.split_fraction
            )
            if self.cfg.show_debug:
                cv2.imshow('Lane Node', vis)
                cv2.imshow('Strip | Mask', bottom_row)
                cv2.waitKey(1)
            if should_save:
                tag = 'nolines' if no_lines else 'periodic'
                stem = f'{self._frame_num:06d}_{tag}'
                cv2.imwrite(str(self._save_debug_dir / f'{stem}_vis.jpg'), vis)
                cv2.imwrite(str(self._save_debug_dir / f'{stem}_mask.jpg'), bottom_row)

    def _drive(self, error: float):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()

        if abs(error) > self.cfg.max_error_to_drive:
            self.get_logger().warn(
                f'Error too large ({error:.2f}), stopping.',
                throttle_duration_sec=1.0)
            self.controller.reset()
            self._cmd_pub.publish(twist)
            return

        twist.twist.linear.x = self.cfg.drive_speed
        twist.twist.angular.z = self.controller.compute(error)
        self._cmd_pub.publish(twist)

    def _stop(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        self._cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LaneKeepingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node._stop()
        node.logger.close()
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
