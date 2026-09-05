#!/usr/bin/env python3
"""Expose the MoveIt GripperCommand Action from WSL and forward it to Isaac."""
import argparse
import math
import os
import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Float64


class GripperActionProxy:
    def __init__(self, action_name, command_topic, state_topic):
        self.node = rclpy.create_node(
            "fr3_gripper_action_proxy",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.position = 0.0
        self._last_state_time = 0.0
        self._position_lock = threading.Lock()
        self._command_publisher = self.node.create_publisher(Float64, command_topic, 1)
        group = ReentrantCallbackGroup()
        self.node.create_subscription(Float64, state_topic, self._on_state, 10, callback_group=group)
        self._server = ActionServer(
            self.node,
            GripperCommand,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._validate_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )

    def _on_state(self, message):
        with self._position_lock:
            self.position = message.data
            self._last_state_time = time.monotonic()

    @staticmethod
    def _validate_goal(goal):
        return GoalResponse.ACCEPT if math.isfinite(goal.command.position) and 0.0 <= goal.command.position <= 0.8 else GoalResponse.REJECT

    def _execute(self, goal_handle):
        target = goal_handle.request.command.position
        deadline = time.monotonic() + 15.0
        best_error = float("inf")
        last_progress = time.monotonic()
        feedback = GripperCommand.Feedback()

        while time.monotonic() < deadline:
            self._command_publisher.publish(Float64(data=target))
            with self._position_lock:
                position = self.position
                last_state_time = self._last_state_time

            reached = abs(position - target) <= 0.01
            error = abs(position - target)
            now = time.monotonic()
            if error + 0.005 < best_error:
                best_error = error
                last_progress = now
            stalled = not reached and now - last_state_time <= 0.5 and now - last_progress >= 1.0
            feedback.position = position
            feedback.effort = 0.0
            feedback.stalled = stalled
            feedback.reached_goal = reached
            goal_handle.publish_feedback(feedback)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(position, False)
            if reached:
                goal_handle.succeed()
                return self._result(position, True)
            if stalled:
                goal_handle.succeed()
                return self._result(position, False, True)
            time.sleep(0.05)

        goal_handle.abort()
        return self._result(position, False)

    @staticmethod
    def _result(position, reached_goal, stalled=False):
        result = GripperCommand.Result()
        result.position = position
        result.effort = 0.0
        result.stalled = stalled
        result.reached_goal = reached_goal
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--action", default="/robotiq_gripper_controller/gripper_cmd")
    parser.add_argument("--command-topic", default="/robotiq_gripper_controller/position_command")
    parser.add_argument("--state-topic", default="/robotiq_gripper_controller/position_state")
    parser.add_argument("--domain-id", type=int, default=42)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    rclpy.init(args=None)
    proxy = GripperActionProxy(args.action, args.command_topic, args.state_topic)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(proxy.node)
    try:
        print(f"[gripper] Action ready: {args.action}", flush=True)
        executor.spin()
    finally:
        executor.shutdown()
        proxy.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
