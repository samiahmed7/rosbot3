#!/usr/bin/env python3
"""One-shot latched publisher: reads the recorded trajectory CSV and publishes
it as a nav_msgs/Path for RViz visualization (debug tool, not part of the
driving stack)."""

import pathlib
import csv
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

WORKSPACE_ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_TRAJECTORY = WORKSPACE_ROOT / 'config' / 'smoothed_trajectory_izhan.csv'


class TrajectoryPathPublisher(Node):

    def __init__(self, trajectory_file: str):
        super().__init__('trajectory_path_publisher')

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self._pub = self.create_publisher(Path, '/trajectory_path', qos)

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        with open(trajectory_file) as f:
            reader = csv.DictReader(f)
            for row in reader:
                x, y, theta = float(row['x']), float(row['y']), float(row['theta'])
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.z = math.sin(theta / 2.0)
                pose.pose.orientation.w = math.cos(theta / 2.0)
                path.poses.append(pose)

        self._pub.publish(path)
        self.get_logger().info(f'Published {len(path.poses)} waypoints from {trajectory_file} to /trajectory_path')


def main(args=None):
    rclpy.init(args=args)
    trajectory_file = sys.argv[1] if len(sys.argv) > 1 else str(DEFAULT_TRAJECTORY)
    node = TrajectoryPathPublisher(trajectory_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
