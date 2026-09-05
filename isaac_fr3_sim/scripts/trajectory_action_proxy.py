#!/usr/bin/env python3
"""Expose MoveIt's FollowJointTrajectory action and forward it to Isaac."""
import argparse
import math
import os
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = ("j1", "j2", "j3", "j4", "j5", "j6")
STATE_MAX_AGE_SEC = 2.0
FINAL_TOLERANCE_RAD = 0.01
SETTLE_SIM_TIME_SEC = 2.0


class TrajectoryActionProxy:
    def __init__(self, action_name, trajectory_topic, state_topic):
        self.node = rclpy.create_node(
            "fr3_trajectory_action_proxy",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self._positions = {}
        self._sim_time = None
        self._last_complete_state_wall = None
        self._lock = threading.Lock()
        group = ReentrantCallbackGroup()
        self._publisher = self.node.create_publisher(JointTrajectory, trajectory_topic, 1)
        self.node.create_subscription(JointState, state_topic, self._on_state, 10, callback_group=group)
        self._server = ActionServer(
            self.node, FollowJointTrajectory, action_name,
            execute_callback=self._execute, goal_callback=self._validate_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group,
        )

    def _on_state(self, message):
        observed = dict(zip(message.name, message.position))
        if not all(name in observed and math.isfinite(observed[name]) for name in ARM_JOINTS):
            return

        with self._lock:
            self._positions = {name: observed[name] for name in ARM_JOINTS}
            self._sim_time = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            self._last_complete_state_wall = time.monotonic()

    def _fresh_arm_state(self):
        with self._lock:
            current = [self._positions.get(name) for name in ARM_JOINTS]
            last_complete_state_wall = self._last_complete_state_wall

        missing = [name for name, position in zip(ARM_JOINTS, current) if position is None]
        if missing:
            return None, f"missing joint state: {', '.join(missing)}"

        age = time.monotonic() - last_complete_state_wall
        if age > STATE_MAX_AGE_SEC:
            return None, f"joint state stale for {age:.2f}s"

        return current, None

    def _validate_goal(self, goal):
        trajectory = goal.trajectory
        if len(trajectory.joint_names) != len(ARM_JOINTS) or set(trajectory.joint_names) != set(ARM_JOINTS):
            return GoalResponse.REJECT
        previous_time = -1.0
        for point in trajectory.points:
            point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            if len(point.positions) != len(ARM_JOINTS) or point_time < 0.0 or point_time <= previous_time:
                return GoalResponse.REJECT
            if not all(math.isfinite(position) for position in point.positions):
                return GoalResponse.REJECT
            previous_time = point_time
        if not trajectory.points:
            return GoalResponse.REJECT
        _, state_error = self._fresh_arm_state()
        if state_error:
            print(f"[trajectory] rejected action goal: {state_error}", flush=True)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        arm_current, state_error = self._fresh_arm_state()
        if state_error:
            print(f"[trajectory] aborted before forwarding: {state_error}", flush=True)
            goal_handle.abort()
            return self._result(FollowJointTrajectory.Result.INVALID_GOAL, state_error)
        current_by_name = dict(zip(ARM_JOINTS, arm_current))
        current = [current_by_name[name] for name in trajectory.joint_names]
        # Isaac's bridge validates and interpolates a complete trajectory.
        # Keep MoveIt's intermediate waypoints so Cartesian and collision-aware
        # planning is not replaced by one large direct joint-space move.
        wire_trajectory = JointTrajectory()
        wire_trajectory.joint_names = list(trajectory.joint_names)
        wire_trajectory.points = [
            JointTrajectoryPoint(positions=list(point.positions), time_from_start=point.time_from_start)
            for point in trajectory.points
            if point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 > 1e-6
        ]
        if not wire_trajectory.points:
            point = trajectory.points[-1]
            wire_trajectory.points = [
                JointTrajectoryPoint(positions=list(point.positions), time_from_start=point.time_from_start)
            ]
        target = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        max_delta = max(abs(goal - actual) for goal, actual in zip(trajectory.points[-1].positions, current))
        raw_duration = (
            wire_trajectory.points[-1].time_from_start.sec
            + wire_trajectory.points[-1].time_from_start.nanosec * 1e-9
        )
        first_time = (
            wire_trajectory.points[0].time_from_start.sec
            + wire_trajectory.points[0].time_from_start.nanosec * 1e-9
        )
        time_offset = max(0.1 - first_time, 0.0)
        base_duration = raw_duration + time_offset
        duration = max(base_duration, 2.0, max_delta / 0.01)
        wall_deadline = time.monotonic() + max(60.0, duration * 5.0 + 20.0)
        scale = duration / base_duration
        for point in wire_trajectory.points:
            point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            scaled_time_ns = round((point_time + time_offset) * scale * 1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(scaled_time_ns, 1_000_000_000)
        with self._lock:
            start_sim_time = self._sim_time
        previous_time = 0.0
        for point in wire_trajectory.points:
            point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            segment_duration = max(0.1, point_time - previous_time)
            segment_time = point.time_from_start.__class__()
            segment_time.sec, segment_time.nanosec = divmod(round(segment_duration * 1e9), 1_000_000_000)
            segment = JointTrajectory()
            segment.joint_names = list(wire_trajectory.joint_names)
            segment.points = [
                JointTrajectoryPoint(
                    positions=list(point.positions),
                    time_from_start=segment_time,
                )
            ]
            self._publisher.publish(segment)
            with self._lock:
                segment_start_time = self._sim_time
            while True:
                with self._lock:
                    sim_time = self._sim_time
                if sim_time is not None and segment_start_time is not None and sim_time - segment_start_time >= segment_duration:
                    break
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "trajectory canceled")
                if time.monotonic() >= wall_deadline:
                    goal_handle.abort()
                    return self._result(FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, "Isaac did not advance through trajectory")
                time.sleep(0.05)
            previous_time = point_time
        # Isaac can run slower than wall-clock time on the available GPU.
        # Judge the requested trajectory against the simulation clock instead.
        final_target = ", ".join(f"{name}={target[name]:.4f}" for name in trajectory.joint_names)
        print(
            f"[trajectory] forwarded full trajectory ({len(trajectory.points)} -> {len(wire_trajectory.points)}), "
            f"max_delta={max_delta:.4f} rad, duration={duration:.3f}s sim-time, "
            f"final_target=[{final_target}]",
            flush=True,
        )
        feedback = FollowJointTrajectory.Feedback()
        last_max_error = float("inf")
        settled_since = None
        while time.monotonic() < wall_deadline:
            arm_actual, state_error = self._fresh_arm_state()
            if state_error:
                print(f"[trajectory] aborted while waiting: {state_error}", flush=True)
                goal_handle.abort()
                return self._result(FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, state_error)
            actual_by_name = dict(zip(ARM_JOINTS, arm_actual))
            with self._lock:
                sim_time = self._sim_time
            actual = [actual_by_name[name] for name in trajectory.joint_names]
            errors = [target[name] - position for name, position in zip(trajectory.joint_names, actual)]
            last_max_error = max(map(abs, errors))
            feedback.joint_names = list(trajectory.joint_names)
            feedback.actual.positions = actual
            feedback.desired.positions = list(trajectory.points[-1].positions)
            feedback.error.positions = errors
            goal_handle.publish_feedback(feedback)
            sim_elapsed = (sim_time - start_sim_time) if sim_time is not None and start_sim_time is not None else None
            if sim_elapsed is not None and sim_elapsed >= duration:
                if last_max_error <= FINAL_TOLERANCE_RAD:
                    settled_since = sim_elapsed if settled_since is None else settled_since
                    if sim_elapsed - settled_since >= SETTLE_SIM_TIME_SEC:
                        print(
                            f"[trajectory] reached target, max error {last_max_error:.4f} rad "
                            f"after {SETTLE_SIM_TIME_SEC:.1f}s settle",
                            flush=True,
                        )
                        goal_handle.succeed()
                        return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "trajectory reached")
                else:
                    settled_since = None
            if sim_elapsed is not None and sim_elapsed >= duration + 5.0:
                break
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "trajectory canceled")
            time.sleep(0.05)
        print(f"[trajectory] timeout, final max error {last_max_error:.4f} rad", flush=True)
        goal_handle.abort()
        return self._result(FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED, "Isaac did not reach the final waypoint")

    @staticmethod
    def _result(code, message):
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--action", default="/fairino3_controller/follow_joint_trajectory")
    parser.add_argument("--trajectory-topic", default="/fairino3_controller/joint_trajectory")
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--domain-id", type=int, default=42)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    rclpy.init(args=None)
    proxy = TrajectoryActionProxy(args.action, args.trajectory_topic, args.state_topic)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(proxy.node)
    try:
        print(f"[trajectory] Action ready: {args.action}", flush=True)
        executor.spin()
    finally:
        executor.shutdown()
        proxy.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
