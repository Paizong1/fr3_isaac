#!/usr/bin/env python3
"""Send one standard GripperCommand goal to the WSL proxy."""
import argparse
import os

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--position", type=float, default=0.2)
    parser.add_argument("--domain-id", type=int, default=42)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    rclpy.init()
    node = rclpy.create_node("fr3_gripper_smoke_client")
    client = ActionClient(node, GripperCommand, "/robotiq_gripper_controller/gripper_cmd")
    try:
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError("gripper Action server is unavailable")
        goal = GripperCommand.Goal()
        goal.command.position = args.position
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("gripper goal was rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=12)
        result = result_future.result()
        if result is None or not result.result.reached_goal:
            raise RuntimeError("gripper did not reach its target")
        print(f"[gripper-test] reached {result.result.position:.3f} rad")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
