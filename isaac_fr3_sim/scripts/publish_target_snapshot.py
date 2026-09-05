#!/usr/bin/env python3
"""Publish one calibrated RGB-D target pose with current simulation timestamps."""
import argparse
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--topic", default="/yolo/target_pose")
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--domain-id", type=int, default=42)
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in (args.x, args.y, args.z)) or args.rate <= 0:
        parser.error("x, y, z must be finite and rate must be positive")

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    rclpy.init(args=None)
    node = rclpy.create_node(
        "fr3_target_snapshot",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    snapshot_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(PoseStamped, args.topic, snapshot_qos)
    pose = PoseStamped()
    pose.header.frame_id = args.frame_id
    pose.pose.position.x = args.x
    pose.pose.position.y = args.y
    pose.pose.position.z = args.z
    pose.pose.orientation.w = 1.0

    def publish():
        pose.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(pose)

    node.create_timer(1.0 / args.rate, publish)
    node.get_logger().info(
        f"Publishing snapshot {args.topic}: ({args.x:.6f}, {args.y:.6f}, {args.z:.6f}) "
        f"in {args.frame_id} at {args.rate:.1f} Hz"
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
