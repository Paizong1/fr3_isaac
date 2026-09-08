#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

namespace
{
using GripperCommand = control_msgs::action::GripperCommand;

double wrap_delta_if_needed(
  double from, double to, const std::string & joint_name, const std::vector<std::string> & wrap_joints)
{
  for (const auto & j : wrap_joints) {
    if (j == joint_name) {
      return std::remainder(to - from, 6.283185307179586);
    }
  }
  return to - from;
}

double plan_cost_joint_l1(
  const moveit::planning_interface::MoveGroupInterface::Plan & plan, const std::vector<std::string> & wrap_joints)
{
  const auto & jt = plan.trajectory_.joint_trajectory;
  if (jt.points.size() < 2 || jt.joint_names.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double cost = 0.0;
  for (size_t i = 1; i < jt.points.size(); ++i) {
    const auto & a = jt.points[i - 1].positions;
    const auto & b = jt.points[i].positions;
    if (a.size() != jt.joint_names.size() || b.size() != jt.joint_names.size()) {
      continue;
    }
    for (size_t j = 0; j < jt.joint_names.size(); ++j) {
      cost += std::fabs(wrap_delta_if_needed(a[j], b[j], jt.joint_names[j], wrap_joints));
    }
  }
  return cost;
}

double plan_cost_time(const moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  const auto & jt = plan.trajectory_.joint_trajectory;
  if (jt.points.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const auto & t = jt.points.back().time_from_start;
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

bool plan_best_of_n(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & move_group,
  int best_of,
  const std::string & metric,
  const std::vector<std::string> & wrap_joints,
  moveit::planning_interface::MoveGroupInterface::Plan & best_plan_out,
  moveit::core::MoveItErrorCode & best_ret_out,
  double time_budget_sec = 0.0)
{
  const int n = std::max(1, best_of);
  bool have_best = false;
  double best_cost = std::numeric_limits<double>::infinity();
  moveit::planning_interface::MoveGroupInterface::Plan best_plan;
  moveit::core::MoveItErrorCode best_ret = moveit::core::MoveItErrorCode::FAILURE;

  const auto t0 = std::chrono::steady_clock::now();

  for (int i = 0; i < n; ++i) {
    if (time_budget_sec > 1e-9) {
      const double elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count();
      if (elapsed >= time_budget_sec) {
        break;
      }
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto ret = move_group.plan(plan);
    if (ret != moveit::core::MoveItErrorCode::SUCCESS) {
      best_ret = ret;
      continue;
    }
    const double cost =
      (metric == "time") ? plan_cost_time(plan) : plan_cost_joint_l1(plan, wrap_joints);
    if (!have_best || cost < best_cost) {
      have_best = true;
      best_cost = cost;
      best_plan = std::move(plan);
      best_ret = ret;
    }
  }

  best_plan_out = std::move(best_plan);
  best_ret_out = best_ret;
  if (have_best) {
    RCLCPP_INFO(
      node->get_logger(), "Best plan selected: metric=%s cost=%.6f (best_of=%d)", metric.c_str(), best_cost, n);
    return true;
  }
  return false;
}

template <typename T>
T declare_or_get_parameter(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const T & default_value)
{
  if (node->has_parameter(name)) {
    try {
      T value{};
      node->get_parameter(name, value);
      return value;
    } catch (const rclcpp::exceptions::InvalidParameterTypeException &) {
      rclcpp::Parameter p;
      node->get_parameter(name, p);
      if constexpr (std::is_same_v<T, double>) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          return static_cast<double>(p.as_int());
        }
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          try {
            return std::stod(p.as_string());
          } catch (...) {
            return default_value;
          }
        }
        return default_value;
      } else if constexpr (std::is_same_v<T, int>) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          return static_cast<int>(p.as_double());
        }
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          try {
            return std::stoi(p.as_string());
          } catch (...) {
            return default_value;
          }
        }
        return default_value;
      } else {
        return default_value;
      }
    }
  }
  return node->declare_parameter<T>(name, default_value);
}

bool execute_plan(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & move_group,
  const moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  const auto ret = move_group.execute(plan);
  if (ret != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execute failed");
    return false;
  }
  return true;
}

bool execute_trajectory(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & move_group,
  const moveit::core::RobotModelConstPtr & robot_model,
  const std::string & group_name,
  moveit::core::RobotState start_state,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  double vel_scale,
  double acc_scale)
{
  move_group.setStartStateToCurrentState();
  if (auto fresh = move_group.getCurrentState(2.0)) {
    start_state = *fresh;
  }
  moveit_msgs::msg::RobotTrajectory timed = trajectory;
  try {
    robot_trajectory::RobotTrajectory rt(robot_model, group_name);
    rt.setRobotTrajectoryMsg(start_state, trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    const double v = std::max(0.0, std::min(1.0, vel_scale));
    const double a = std::max(0.0, std::min(1.0, acc_scale));
    const bool ok = iptp.computeTimeStamps(rt, v, a);
    if (ok) {
      rt.getRobotTrajectoryMsg(timed);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(), "Time parameterization skipped: %s", e.what());
  }

  const auto ret = move_group.execute(timed);
  if (ret != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execute trajectory failed");
    return false;
  }
  return true;
}

bool descend_eef_to_z(
  const rclcpp::Node::SharedPtr & node,
  moveit::planning_interface::MoveGroupInterface & arm,
  const moveit::core::RobotModelConstPtr & robot_model,
  const std::string & arm_group,
  const std::string & eef_link,
  double target_z,
  double min_eef_z,
  double eef_step,
  double min_fraction,
  bool avoid_collisions,
  double vel_scale,
  double acc_scale,
  double current_state_timeout,
  const char * reason)
{
  arm.setStartStateToCurrentState();
  const auto pose_now = arm.getCurrentPose(eef_link).pose;
  const double clamped_z = std::max(target_z, min_eef_z);
  const double current_z = static_cast<double>(pose_now.position.z);
  if (current_z <= clamped_z + 0.002) {
    return true;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "%s: descend z %.4f -> %.4f (delta=%.4f m, avoid_collisions=%s)",
    reason,
    current_z,
    clamped_z,
    current_z - clamped_z,
    avoid_collisions ? "true" : "false");

  geometry_msgs::msg::Pose p0 = pose_now;
  geometry_msgs::msg::Pose p1 = pose_now;
  p1.position.z = clamped_z;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(p0);
  waypoints.push_back(p1);

  moveit_msgs::msg::RobotTrajectory traj;
  arm.setStartStateToCurrentState();
  const double frac = arm.computeCartesianPath(waypoints, eef_step, 0.0, traj, avoid_collisions);
  if (frac >= std::max(0.0, std::min(1.0, min_fraction))) {
    auto current_state_ptr = arm.getCurrentState(current_state_timeout);
    if (current_state_ptr) {
      if (execute_trajectory(node, arm, robot_model, arm_group, *current_state_ptr, traj, vel_scale, acc_scale)) {
        return true;
      }
    }
  }

  arm.setStartStateToCurrentState();
  arm.clearPoseTargets();
  arm.setPositionTarget(
    static_cast<double>(p1.position.x),
    static_cast<double>(p1.position.y),
    clamped_z,
    eef_link);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(node->get_logger(), "%s: pose plan to z=%.4f failed", reason, clamped_z);
    return false;
  }
  return execute_plan(node, arm, plan);
}

bool send_gripper_action(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp_action::Client<GripperCommand>::SharedPtr & client,
  double position,
  double max_effort,
  const std::chrono::milliseconds timeout,
  bool wait_result,
  bool timeout_is_success,
  bool * stalled_out = nullptr)
{
  if (!client) {
    return false;
  }
  if (!client->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action server not available");
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = max_effort;

  auto future_goal = client->async_send_goal(goal);
  if (future_goal.wait_for(timeout) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "Gripper goal send timeout");
    return false;
  }
  auto goal_handle = future_goal.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Gripper goal rejected");
    return false;
  }

  if (!wait_result) {
    return true;
  }

  auto future_result = client->async_get_result(goal_handle);
  if (future_result.wait_for(timeout) != std::future_status::ready) {
    if (timeout_is_success) {
      RCLCPP_WARN(node->get_logger(), "Gripper result timeout, continue");
      return true;
    }
    RCLCPP_ERROR(node->get_logger(), "Gripper result timeout");
    return false;
  }
  const auto wrapped = future_result.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node->get_logger(), "Gripper action failed");
    return false;
  }
  if (stalled_out) {
    *stalled_out = wrapped.result->stalled;
  }
  return true;
}

bool send_gripper_action_slow_close(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp_action::Client<GripperCommand>::SharedPtr & client,
  double open_position,
  double close_position,
  double max_effort,
  int steps,
  const std::chrono::milliseconds per_goal_timeout,
  const std::chrono::milliseconds step_delay,
  bool wait_result,
  bool timeout_is_success)
{
  const int n = std::max(1, steps);
  const double delta = close_position - open_position;
  for (int i = 1; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    const double pos = open_position + delta * t;
    const bool is_last = (i == n);
    const bool wait_this_step = wait_result;
    const auto step_timeout =
      is_last ? per_goal_timeout : std::chrono::milliseconds(std::min<int64_t>(2000, per_goal_timeout.count()));
    bool stalled = false;
    if (!send_gripper_action(
          node, client, pos, max_effort, step_timeout, wait_this_step, timeout_is_success, &stalled)) {
      return false;
    }
    if (stalled) {
      RCLCPP_INFO(node->get_logger(), "Gripper contact at %.3f rad; holding position", pos);
      return true;
    }
    if (i != n && step_delay.count() > 0) {
      std::this_thread::sleep_for(step_delay);
    }
  }
  return true;
}

geometry_msgs::msg::PoseStamped transform_pose(
  const rclcpp::Node::SharedPtr & node,
  tf2_ros::Buffer & tf_buffer,
  const geometry_msgs::msg::PoseStamped & in,
  const std::string & target_frame)
{
  if (in.header.frame_id == target_frame) {
    return in;
  }
  geometry_msgs::msg::PoseStamped out;
  const auto tf = tf_buffer.lookupTransform(target_frame, in.header.frame_id, tf2::TimePointZero);
  tf2::doTransform(in, out, tf);
  out.header.frame_id = target_frame;
  if (out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0) {
    out.header.stamp = node->now();
  }
  return out;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("pick_banana_node", node_options);

  auto main_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto action_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  const std::string target_pose_topic =
    declare_or_get_parameter<std::string>(node, "target_pose_topic", "/yolo/target_pose");
  const std::string grasp_state_topic =
    declare_or_get_parameter<std::string>(node, "grasp_state_topic", "/world_model/grasp_state");
  const std::string target_object_id =
    declare_or_get_parameter<std::string>(node, "target_object_id", "banana-1");
  const std::string joint_states_topic =
    declare_or_get_parameter<std::string>(node, "joint_states_topic", "/joint_states");
  const std::string arm_group =
    declare_or_get_parameter<std::string>(node, "arm_group", "fairino3_v6_group");
  const std::string gripper_group =
    declare_or_get_parameter<std::string>(node, "gripper_group", "gripper");
  const std::string gripper_open_named_target =
    declare_or_get_parameter<std::string>(node, "gripper_open", "open");
  const std::string gripper_close_named_target =
    declare_or_get_parameter<std::string>(node, "gripper_close", "close");
  const bool use_direct_gripper_action = declare_or_get_parameter<bool>(node, "use_direct_gripper_action", true);
  const std::string gripper_action_name =
    declare_or_get_parameter<std::string>(node, "gripper_action_name", "/robotiq_gripper_controller/gripper_cmd");
  const double gripper_open_pos = declare_or_get_parameter<double>(node, "gripper_open_pos", 0.0);
  const double gripper_close_pos = declare_or_get_parameter<double>(node, "gripper_close_pos", 0.75);
  const double gripper_max_effort = declare_or_get_parameter<double>(node, "gripper_max_effort", 100.0);
  const std::string gripper_joint_name =
    declare_or_get_parameter<std::string>(node, "gripper_joint_name", "robotiq_85_left_knuckle_joint");
  const int gripper_action_timeout_ms = declare_or_get_parameter<int>(node, "gripper_action_timeout_ms", 15000);
  const bool gripper_slow_close = declare_or_get_parameter<bool>(node, "gripper_slow_close", false);
  const int gripper_close_steps = declare_or_get_parameter<int>(node, "gripper_close_steps", 12);
  const int gripper_close_step_delay_ms = declare_or_get_parameter<int>(node, "gripper_close_step_delay_ms", 350);
  const bool gripper_wait_result = declare_or_get_parameter<bool>(node, "gripper_wait_result", true);
  const bool gripper_timeout_is_success = declare_or_get_parameter<bool>(node, "gripper_timeout_is_success", false);
  const int pre_gripper_close_pause_ms = declare_or_get_parameter<int>(node, "pre_gripper_close_pause_ms", 800);
  const int post_gripper_close_settle_ms = declare_or_get_parameter<int>(node, "post_gripper_close_settle_ms", 3000);
  const bool lift_after_close_enable = declare_or_get_parameter<bool>(node, "lift_after_close_enable", false);
  const bool staged_lift_enable = declare_or_get_parameter<bool>(node, "staged_lift_enable", true);
  const double staged_lift_first_step = declare_or_get_parameter<double>(node, "staged_lift_first_step", 0.02);
  const int staged_lift_pause_ms = declare_or_get_parameter<int>(node, "staged_lift_pause_ms", 800);

  const double pregrasp_z_offset = declare_or_get_parameter<double>(node, "pregrasp_z_offset", 0.08);
  const double grasp_z_offset = declare_or_get_parameter<double>(node, "grasp_z_offset", 0.010);
  const double finger_tip_z_offset = declare_or_get_parameter<double>(node, "finger_tip_z_offset", 0.010);
  const double lift_z_offset = declare_or_get_parameter<double>(node, "lift_z_offset", 0.18);
  const double eef_step = declare_or_get_parameter<double>(node, "eef_step", 0.01);
  const double min_fraction = declare_or_get_parameter<double>(node, "min_fraction", 0.6);
  const bool avoid_collisions = declare_or_get_parameter<bool>(node, "avoid_collisions", false);
  const bool descend_avoid_collisions =
    declare_or_get_parameter<bool>(node, "descend_avoid_collisions", false);
  const double reach_grasp_tolerance = declare_or_get_parameter<double>(node, "reach_grasp_tolerance", 0.05);
  const double vel_scale = declare_or_get_parameter<double>(node, "vel_scale", 0.3);
  const double acc_scale = declare_or_get_parameter<double>(node, "acc_scale", 0.3);
  const double lift_vel_scale = declare_or_get_parameter<double>(node, "lift_vel_scale", 0.05);
  const double lift_acc_scale = declare_or_get_parameter<double>(node, "lift_acc_scale", 0.05);
  const double planning_time = declare_or_get_parameter<double>(node, "planning_time", 3.0);
  const int planning_attempts = declare_or_get_parameter<int>(node, "planning_attempts", 5);
  const int best_of_plans = declare_or_get_parameter<int>(node, "best_of_plans", 5);
  const std::string best_plan_metric =
    declare_or_get_parameter<std::string>(node, "best_plan_metric", "joint_l1");
  const std::string planning_pipeline_id =
    declare_or_get_parameter<std::string>(node, "planning_pipeline_id", "ompl");
  const double current_state_timeout = declare_or_get_parameter<double>(node, "current_state_timeout", 5.0);
  const bool execute_once = declare_or_get_parameter<bool>(node, "execute_once", true);
  const bool plan_only = declare_or_get_parameter<bool>(node, "plan_only", false);
  const bool inspect_after_pregrasp =
    declare_or_get_parameter<bool>(node, "inspect_after_pregrasp", false);
  const bool inspect_after_grasp =
    declare_or_get_parameter<bool>(node, "inspect_after_grasp", false);
  const bool inspect_after_close =
    declare_or_get_parameter<bool>(node, "inspect_after_close", false);
  const bool pregrasp_use_identity_orientation =
    declare_or_get_parameter<bool>(node, "pregrasp_use_identity_orientation", false);
  const std::string pregrasp_orientation_mode =
    declare_or_get_parameter<std::string>(node, "pregrasp_orientation_mode", "");
  const std::vector<double> pregrasp_fixed_rpy =
    declare_or_get_parameter<std::vector<double>>(node, "pregrasp_fixed_rpy", std::vector<double>{0.0, 0.0, 0.0});
  const double pregrasp_yaw_offset = declare_or_get_parameter<double>(node, "pregrasp_yaw_offset", 0.0);
  const double goal_pos_tolerance = declare_or_get_parameter<double>(node, "goal_pos_tolerance", 0.01);
  const double goal_ori_tolerance = declare_or_get_parameter<double>(node, "goal_ori_tolerance", 0.2);
  const bool pregrasp_fallback_enable = declare_or_get_parameter<bool>(node, "pregrasp_fallback_enable", true);
  const double pregrasp_fallback_ori_tolerance =
    declare_or_get_parameter<double>(node, "pregrasp_fallback_ori_tolerance", 1.57);
  const bool pregrasp_fallback_position_only =
    declare_or_get_parameter<bool>(node, "pregrasp_fallback_position_only", true);
  const double pregrasp_fallback_planning_time =
    declare_or_get_parameter<double>(node, "pregrasp_fallback_planning_time", 10.0);
  const int pregrasp_fallback_planning_attempts =
    declare_or_get_parameter<int>(node, "pregrasp_fallback_planning_attempts", 10);
  const bool pregrasp_orientation_enforce =
    declare_or_get_parameter<bool>(node, "pregrasp_orientation_enforce", false);
  const double pregrasp_orientation_enforce_min_angle =
    declare_or_get_parameter<double>(node, "pregrasp_orientation_enforce_min_angle", 0.08);
  const double pregrasp_orientation_enforce_planning_time =
    declare_or_get_parameter<double>(node, "pregrasp_orientation_enforce_planning_time", 2.0);
  const int pregrasp_orientation_enforce_attempts =
    declare_or_get_parameter<int>(node, "pregrasp_orientation_enforce_attempts", 5);
  const double pregrasp_orientation_enforce_ori_tolerance =
    declare_or_get_parameter<double>(node, "pregrasp_orientation_enforce_ori_tolerance", 0.05);
  const double min_target_age_sec = declare_or_get_parameter<double>(node, "min_target_age_sec", 0.2);
  const double target_pos_epsilon = declare_or_get_parameter<double>(node, "target_pos_epsilon", 0.01);
  const double retry_interval_sec = declare_or_get_parameter<double>(node, "retry_interval_sec", 1.0);
  const bool refine_at_pregrasp_enable = declare_or_get_parameter<bool>(node, "refine_at_pregrasp_enable", true);
  const bool refine_at_pregrasp_cartesian = declare_or_get_parameter<bool>(node, "refine_at_pregrasp_cartesian", true);
  const double refine_at_pregrasp_min_xy = declare_or_get_parameter<double>(node, "refine_at_pregrasp_min_xy", 0.004);
  const int closed_loop_max_iterations =
    declare_or_get_parameter<int>(node, "closed_loop_max_iterations", 5);
  const double closed_loop_xyz_tolerance =
    declare_or_get_parameter<double>(node, "closed_loop_xyz_tolerance", 0.007);
  const double closed_loop_orientation_tolerance =
    declare_or_get_parameter<double>(node, "closed_loop_orientation_tolerance", 0.05);
  const double closed_loop_max_translation_step =
    declare_or_get_parameter<double>(node, "closed_loop_max_translation_step", 0.01);
  const double closed_loop_max_orientation_step =
    declare_or_get_parameter<double>(node, "closed_loop_max_orientation_step", 0.05);
  const double target_z_override = declare_or_get_parameter<double>(
    node, "target_z_override", std::numeric_limits<double>::quiet_NaN());
  const double target_x_offset = declare_or_get_parameter<double>(node, "target_x_offset", 0.0);
  const double target_y_offset = declare_or_get_parameter<double>(node, "target_y_offset", 0.0);
  const double target_z_offset = declare_or_get_parameter<double>(node, "target_z_offset", 0.0);
  const double min_grasp_height_above_target =
    declare_or_get_parameter<double>(node, "min_grasp_height_above_target", 0.0);
  const bool elbow_constraint_enable = declare_or_get_parameter<bool>(node, "elbow_constraint_enable", false);
  const std::string elbow_joint_name = declare_or_get_parameter<std::string>(node, "elbow_joint_name", "j3");
  const double elbow_target = declare_or_get_parameter<double>(
    node, "elbow_target", std::numeric_limits<double>::quiet_NaN());
  const double elbow_delta = declare_or_get_parameter<double>(node, "elbow_delta", 0.0);
  const double elbow_tolerance = declare_or_get_parameter<double>(node, "elbow_tolerance", 0.35);
  const bool posture_constraint_enable = declare_or_get_parameter<bool>(node, "posture_constraint_enable", false);
  const std::vector<std::string> posture_joint_names =
    declare_or_get_parameter<std::vector<std::string>>(node, "posture_joint_names", std::vector<std::string>{});
  const std::vector<double> posture_joint_deltas =
    declare_or_get_parameter<std::vector<double>>(node, "posture_joint_deltas", std::vector<double>{});
  const std::vector<double> posture_joint_tolerances =
    declare_or_get_parameter<std::vector<double>>(node, "posture_joint_tolerances", std::vector<double>{});
  const double posture_default_tolerance = declare_or_get_parameter<double>(node, "posture_default_tolerance", 0.5);
  const bool ik_bias_enable = declare_or_get_parameter<bool>(node, "ik_bias_enable", true);
  const double ik_bias_timeout = declare_or_get_parameter<double>(node, "ik_bias_timeout", 0.15);
  const int ik_bias_attempts = declare_or_get_parameter<int>(node, "ik_bias_attempts", 8);
  const double ik_bias_noise = declare_or_get_parameter<double>(node, "ik_bias_noise", 0.25);
  const std::vector<std::string> wrap_joints =
    declare_or_get_parameter<std::vector<std::string>>(node, "wrap_joints", std::vector<std::string>{"j1", "j6"});
  const bool add_table_collision = declare_or_get_parameter<bool>(node, "add_table_collision", true);
  const std::string table_frame = declare_or_get_parameter<std::string>(node, "table_frame", "base_link");
  const std::vector<double> table_size =
    declare_or_get_parameter<std::vector<double>>(node, "table_size", std::vector<double>{5.0, 5.0, 0.4});
  const std::vector<double> table_center =
    declare_or_get_parameter<std::vector<double>>(node, "table_center", std::vector<double>{0.0, 0.0, -0.2});
  const bool auto_final_descend_enable = declare_or_get_parameter<bool>(node, "auto_final_descend_enable", false);
  const double auto_final_descend_margin = declare_or_get_parameter<double>(node, "auto_final_descend_margin", 0.001);
  const double auto_final_descend_max = declare_or_get_parameter<double>(node, "auto_final_descend_max", 0.025);
  const double force_extra_descend = declare_or_get_parameter<double>(node, "force_extra_descend", 0.0);
  const double min_eef_z = declare_or_get_parameter<double>(node, "min_eef_z", -0.08);
  const double descend_below_target = declare_or_get_parameter<double>(node, "descend_below_target", 0.0);
  const bool single_descend_enable = declare_or_get_parameter<bool>(node, "single_descend_enable", false);
  const bool single_descend_close_gripper =
    declare_or_get_parameter<bool>(node, "single_descend_close_gripper", false);
  const double single_descend_max_distance =
    declare_or_get_parameter<double>(node, "single_descend_max_distance", 0.01);
  const double single_descend_min_height_above_target =
    declare_or_get_parameter<double>(node, "single_descend_min_height_above_target", 0.08);
  const double best_of_time_budget_sec = declare_or_get_parameter<double>(node, "best_of_time_budget_sec", 6.0);
  const bool lift_only = declare_or_get_parameter<bool>(node, "lift_only", false);
  const double lift_only_distance = declare_or_get_parameter<double>(node, "lift_only_distance", 0.02);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  moveit::planning_interface::MoveGroupInterface arm(node, arm_group);
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper;
  if (!use_direct_gripper_action) {
    gripper = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node, gripper_group);
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  if (add_table_collision) {
    moveit_msgs::msg::CollisionObject table;
    table.id = "table_slab";
    table.header.frame_id = table_frame.empty() ? arm.getPlanningFrame() : table_frame;
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    const double sx = table_size.size() > 0 ? table_size[0] : 5.0;
    const double sy = table_size.size() > 1 ? table_size[1] : 5.0;
    const double sz = table_size.size() > 2 ? table_size[2] : 0.4;
    prim.dimensions = {sx, sy, sz};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = table_center.size() > 0 ? table_center[0] : 0.0;
    pose.position.y = table_center.size() > 1 ? table_center[1] : 0.0;
    pose.position.z = table_center.size() > 2 ? table_center[2] : -0.2;

    table.primitives.push_back(prim);
    table.primitive_poses.push_back(pose);
    planning_scene_interface.applyCollisionObject(table);
  }

  if (!planning_pipeline_id.empty()) {
    arm.setPlanningPipelineId(planning_pipeline_id);
    if (gripper) {
      gripper->setPlanningPipelineId(planning_pipeline_id);
    }
  }

  arm.setMaxVelocityScalingFactor(std::max(0.0, std::min(1.0, vel_scale)));
  arm.setMaxAccelerationScalingFactor(std::max(0.0, std::min(1.0, acc_scale)));
  arm.setPlanningTime(std::max(0.1, planning_time));
  arm.setNumPlanningAttempts(std::max(1, planning_attempts));
  if (goal_pos_tolerance > 0.0) {
    arm.setGoalPositionTolerance(goal_pos_tolerance);
  }
  if (goal_ori_tolerance > 0.0) {
    arm.setGoalOrientationTolerance(goal_ori_tolerance);
  }

  std::string eef_link = arm.getEndEffectorLink();
  if (eef_link.empty()) {
    eef_link = "ur5e_gripper_tcp";
  }
  arm.setEndEffectorLink(eef_link);

  auto robot_model = arm.getRobotModel();
  if (!robot_model) {
    RCLCPP_ERROR(node->get_logger(), "Robot model not available from MoveGroupInterface");
    rclcpp::shutdown();
    return -1;
  }

  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client;
  if (use_direct_gripper_action) {
    gripper_client = rclcpp_action::create_client<GripperCommand>(node, gripper_action_name, action_cb_group);
  }

  std::mutex joint_mutex;
  sensor_msgs::msg::JointState latest_joint_state;
  bool have_joint_state = false;

  auto js_qos = rclcpp::SensorDataQoS();
  rclcpp::SubscriptionOptions js_sub_opt;
  js_sub_opt.callback_group = action_cb_group;
  auto joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic, js_qos, [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::scoped_lock<std::mutex> lock(joint_mutex);
      latest_joint_state = *msg;
      have_joint_state = true;
    }, js_sub_opt);

  std::mutex target_mutex;
  geometry_msgs::msg::PoseStamped latest_target;
  bool have_target = false;
  std::chrono::steady_clock::time_point target_last_change_steady = std::chrono::steady_clock::now();
  std::atomic_bool executing{false};
  std::atomic_bool done{false};
  std::chrono::steady_clock::time_point last_attempt_steady = std::chrono::steady_clock::now();

  auto grasp_state_pub = node->create_publisher<std_msgs::msg::String>(
    grasp_state_topic, rclcpp::QoS(10).reliable());
  auto publish_grasp_state = [&](const std::string & state, const std::string & reason) {
      const auto stamp = node->now();
      const auto stamp_ns = stamp.nanoseconds();
      std_msgs::msg::String message;
      std::ostringstream payload;
      payload << "{\"object_id\":\"" << target_object_id
              << "\",\"state\":\"" << state
              << "\",\"reason\":\"" << reason
              << "\",\"stamp\":{\"sec\":" << stamp_ns / 1000000000LL
              << ",\"nanosec\":" << stamp_ns % 1000000000LL << "}}";
      message.data = payload.str();
      grasp_state_pub->publish(message);
      RCLCPP_INFO(node->get_logger(), "Grasp state: %s (%s)", state.c_str(), reason.c_str());
    };

  rclcpp::SubscriptionOptions target_sub_opt;
  target_sub_opt.callback_group = main_cb_group;
  auto target_qos = rclcpp::QoS(1).reliable().transient_local();
  auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    target_pose_topic, target_qos,
    [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::scoped_lock<std::mutex> lock(target_mutex);
      const auto now_steady = std::chrono::steady_clock::now();
      if (!have_target) {
        latest_target = *msg;
        target_last_change_steady = now_steady;
        have_target = true;
        return;
      }

      const auto & a = latest_target.pose.position;
      const auto & b = msg->pose.position;
      const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
      const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
      const double dz = static_cast<double>(a.z) - static_cast<double>(b.z);
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist >= target_pos_epsilon) {
        target_last_change_steady = now_steady;
      }
      latest_target = *msg;
    }, target_sub_opt);

  // FK is deliberately rebuilt from the received joint state instead of the
  // MoveIt monitor so the correction starts from Isaac's measured position.
  auto actual_tcp_from_joint_states = [&]() -> std::optional<geometry_msgs::msg::Pose> {
      sensor_msgs::msg::JointState js_copy;
      {
        std::scoped_lock<std::mutex> lock(joint_mutex);
        if (!have_joint_state) {
          return std::nullopt;
        }
        js_copy = latest_joint_state;
      }
      moveit::core::RobotState state(robot_model);
      for (size_t i = 0; i < js_copy.name.size() && i < js_copy.position.size(); ++i) {
        const auto * joint = robot_model->getJointModel(js_copy.name[i]);
        if (joint && joint->getVariableCount() == 1) {
          const double position = static_cast<double>(js_copy.position[i]);
          state.setJointPositions(joint, &position);
        }
      }
      state.update();
      return tf2::toMsg(state.getGlobalLinkTransform(eef_link));
    };

  auto closed_loop_correct = [&](const geometry_msgs::msg::Pose & desired_pose,
                                 const geometry_msgs::msg::PoseStamped & object_pose,
                                 const char * stage) {
      const int max_iterations = std::max(1, std::min(5, closed_loop_max_iterations));
      const double xyz_tolerance = std::max(0.0, closed_loop_xyz_tolerance);
      const double max_translation = std::max(1e-4, closed_loop_max_translation_step);

      for (int iteration = 1; iteration <= max_iterations; ++iteration) {
        const auto actual_pose = actual_tcp_from_joint_states();
        if (!actual_pose) {
          RCLCPP_ERROR(node->get_logger(), "%s closed-loop: no /joint_states", stage);
          return false;
        }

        tf2::Transform actual_tf;
        tf2::Transform desired_tf;
        tf2::Transform object_tf;
        tf2::fromMsg(*actual_pose, actual_tf);
        tf2::fromMsg(desired_pose, desired_tf);
        tf2::fromMsg(object_pose.pose, object_tf);
        const auto target_in_tcp = actual_tf.inverse() * object_tf.getOrigin();
        const auto error_in_tcp = actual_tf.inverse() * desired_tf.getOrigin();
        tf2::Quaternion q_error = actual_tf.getRotation().inverse() * desired_tf.getRotation();
        q_error.normalize();
        const double orientation_error = 2.0 * std::acos(
          std::min(1.0, std::max(0.0, std::fabs(q_error.w()))));

        RCLCPP_INFO(
          node->get_logger(),
          "%s closed-loop %d/%d: actual_tcp=(%.4f %.4f %.4f) target_in_tcp_xyz=(%.4f %.4f %.4f) "
          "command_error_tcp_xyz=(%.4f %.4f %.4f) orientation_error=%.4f rad",
          stage, iteration, max_iterations,
          static_cast<double>(actual_pose->position.x), static_cast<double>(actual_pose->position.y),
          static_cast<double>(actual_pose->position.z), target_in_tcp.x(), target_in_tcp.y(), target_in_tcp.z(),
          error_in_tcp.x(), error_in_tcp.y(), error_in_tcp.z(), orientation_error);

        if (std::fabs(error_in_tcp.x()) <= xyz_tolerance &&
            std::fabs(error_in_tcp.y()) <= xyz_tolerance &&
            std::fabs(error_in_tcp.z()) <= xyz_tolerance) {
          return true;
        }

        geometry_msgs::msg::Pose step_pose = *actual_pose;
        const double dx = static_cast<double>(desired_pose.position.x) - static_cast<double>(actual_pose->position.x);
        const double dy = static_cast<double>(desired_pose.position.y) - static_cast<double>(actual_pose->position.y);
        const double dz = static_cast<double>(desired_pose.position.z) - static_cast<double>(actual_pose->position.z);
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double scale = iteration == 1 ? 1.0 :
          (distance > max_translation ? max_translation / distance : 1.0);
        step_pose.position.x += dx * scale;
        step_pose.position.y += dy * scale;
        step_pose.position.z += dz * scale;
        // Detection refines object position, not tool attitude.  Keeping the
        // planned attitude avoids repeated in-place joint corrections.

        arm.setStartStateToCurrentState();
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double fraction = arm.computeCartesianPath(
          {*actual_pose, step_pose}, eef_step, 0.0, trajectory, avoid_collisions);
        if (fraction < min_fraction) {
          RCLCPP_ERROR(node->get_logger(), "%s closed-loop: Cartesian fraction %.3f too low", stage, fraction);
          return false;
        }
        const auto current_state_ptr = arm.getCurrentState(current_state_timeout);
        if (!current_state_ptr || !execute_trajectory(
              node, arm, robot_model, arm_group, *current_state_ptr, trajectory, vel_scale, acc_scale)) {
          return false;
        }
      }
      RCLCPP_ERROR(node->get_logger(), "%s closed-loop: not centered after %d iterations", stage, max_iterations);
      return false;
    };

  auto tick = [&]() {
    if (done.load()) {
      return;
    }
    if (executing.exchange(true)) {
      return;
    }
    auto finish_fail = [&](const std::string & reason = "execution_failed") {
      publish_grasp_state("failed", reason);
      if (execute_once) {
        done.store(true);
      }
      executing.store(false);
    };

    if (lift_only) {
      try {
        sensor_msgs::msg::JointState js_copy;
        {
          std::scoped_lock<std::mutex> lock(joint_mutex);
          if (!have_joint_state) {
            executing.store(false);
            return;
          }
          js_copy = latest_joint_state;
        }
        moveit::core::RobotState start_state(robot_model);
        for (size_t i = 0; i < js_copy.name.size() && i < js_copy.position.size(); ++i) {
          const auto * joint = robot_model->getJointModel(js_copy.name[i]);
          if (joint && joint->getVariableCount() == 1) {
            const double position = static_cast<double>(js_copy.position[i]);
            start_state.setJointPositions(joint, &position);
          }
        }
        start_state.update();
        arm.setStartState(start_state);
        const auto current = tf2::toMsg(start_state.getGlobalLinkTransform(eef_link));
        auto lift = current;
        lift.position.z += std::max(0.0, lift_only_distance);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double fraction = arm.computeCartesianPath(
          {current, lift}, eef_step, 0.0, trajectory, descend_avoid_collisions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        if (fraction < min_fraction || !execute_plan(node, arm, plan)) {
          finish_fail("lift_only_failed");
          return;
        }
        publish_grasp_state("lifted", "lift_only_complete");
        done.store(true);
        executing.store(false);
        return;
      } catch (const std::exception & exc) {
        RCLCPP_ERROR(node->get_logger(), "Lift-only failed: %s", exc.what());
        finish_fail("lift_only_exception");
        return;
      }
    }

    geometry_msgs::msg::PoseStamped target_copy;
    std::chrono::steady_clock::time_point target_change_time_copy = std::chrono::steady_clock::now();
    {
      std::scoped_lock<std::mutex> lock(target_mutex);
      if (!have_target) {
        executing.store(false);
        return;
      }
      target_copy = latest_target;
      target_change_time_copy = target_last_change_steady;
    }

    const auto now_steady = std::chrono::steady_clock::now();
    const double settle_age = std::chrono::duration<double>(now_steady - target_change_time_copy).count();
    if (min_target_age_sec > 0.0 && settle_age < min_target_age_sec) {
      executing.store(false);
      return;
    }

    if (retry_interval_sec > 0.0) {
      const double since_last = std::chrono::duration<double>(now_steady - last_attempt_steady).count();
      if (since_last < retry_interval_sec) {
        executing.store(false);
        return;
      }
    }
    last_attempt_steady = now_steady;

    try {
      publish_grasp_state("approach", "target_accepted");
      sensor_msgs::msg::JointState js_copy;
      {
        std::scoped_lock<std::mutex> lock(joint_mutex);
        if (!have_joint_state) {
          executing.store(false);
          return;
        }
        js_copy = latest_joint_state;
      }

      moveit::core::RobotState start_state(robot_model);
      auto set_1d_joint = [&](moveit::core::RobotState & state, const std::string & joint_name, double value) {
        const auto * jm = robot_model->getJointModel(joint_name);
        if (!jm) {
          return;
        }
        if (jm->getVariableCount() != 1) {
          return;
        }
        const double pos = value;
        state.setJointPositions(jm, &pos);
      };

      for (size_t i = 0; i < js_copy.name.size() && i < js_copy.position.size(); ++i) {
        const auto & n = js_copy.name[i];
        const auto * jm = robot_model->getJointModel(n);
        if (!jm) {
          continue;
        }
        if (jm->getVariableCount() != 1) {
          continue;
        }
        const double pos = static_cast<double>(js_copy.position[i]);
        start_state.setJointPositions(jm, &pos);
      }
      set_1d_joint(start_state, gripper_joint_name, gripper_open_pos);
      set_1d_joint(start_state, "robotiq_85_left_inner_knuckle_joint", gripper_open_pos);
      set_1d_joint(start_state, "robotiq_85_right_knuckle_joint", -gripper_open_pos);
      set_1d_joint(start_state, "robotiq_85_right_inner_knuckle_joint", -gripper_open_pos);
      set_1d_joint(start_state, "robotiq_85_left_finger_tip_joint", -gripper_open_pos);
      set_1d_joint(start_state, "robotiq_85_right_finger_tip_joint", gripper_open_pos);
      start_state.update();

      if (!start_state.getRobotModel()) {
        RCLCPP_WARN(node->get_logger(), "Robot state not ready, skip this cycle");
        executing.store(false);
        return;
      }

      const std::string planning_frame = arm.getPlanningFrame();
      const auto target_in_planning = transform_pose(node, tf_buffer, target_copy, planning_frame);

      const Eigen::Isometry3d & eef_tf = start_state.getGlobalLinkTransform(eef_link);
      const geometry_msgs::msg::Pose current_pose = tf2::toMsg(eef_tf);
      const auto & cq = current_pose.orientation;
      RCLCPP_INFO(
        node->get_logger(),
        "Current quat=(%.4f %.4f %.4f %.4f)",
        static_cast<double>(cq.x),
        static_cast<double>(cq.y),
        static_cast<double>(cq.z),
        static_cast<double>(cq.w));
      RCLCPP_INFO(
        node->get_logger(),
        "Attempt: planning_frame=%s eef_link=%s target_frame=%s target_xyz=(%.3f %.3f %.3f) current_xyz=(%.3f %.3f %.3f)",
        planning_frame.c_str(),
        eef_link.c_str(),
        target_in_planning.header.frame_id.c_str(),
        static_cast<double>(target_in_planning.pose.position.x),
        static_cast<double>(target_in_planning.pose.position.y),
        static_cast<double>(target_in_planning.pose.position.z),
        static_cast<double>(current_pose.position.x),
        static_cast<double>(current_pose.position.y),
        static_cast<double>(current_pose.position.z));

      if (!single_descend_enable && use_direct_gripper_action) {
        if (!send_gripper_action(
              node,
              gripper_client,
              gripper_open_pos,
              gripper_max_effort,
              std::chrono::milliseconds(std::max(1000, gripper_action_timeout_ms)),
              gripper_wait_result,
              gripper_timeout_is_success)) {
          finish_fail();
          return;
        }
      } else if (!single_descend_enable) {
        gripper->setStartState(start_state);
        gripper->setNamedTarget(gripper_open_named_target);
        moveit::planning_interface::MoveGroupInterface::Plan gripper_open_plan;
        if (gripper->plan(gripper_open_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          if (!execute_plan(node, *gripper, gripper_open_plan)) {
            finish_fail();
            return;
          }
        }
      }

      arm.setStartStateToCurrentState();
      arm.clearPoseTargets();
      const double tx = static_cast<double>(target_in_planning.pose.position.x) + target_x_offset;
      const double ty = static_cast<double>(target_in_planning.pose.position.y) + target_y_offset;
      double tz_obj = static_cast<double>(target_in_planning.pose.position.z);
      if (!std::isnan(target_z_override)) {
        tz_obj = target_z_override;
      }
      tz_obj += target_z_offset;
      const double tz_pre = tz_obj + pregrasp_z_offset;
      RCLCPP_INFO(
        node->get_logger(),
        "Height check: target_z=%.3f pregrasp_z=%.3f current_z=%.3f (delta_current_to_target=%.3f m)",
        tz_obj,
        tz_pre,
        static_cast<double>(current_pose.position.z),
        static_cast<double>(current_pose.position.z) - tz_obj);
      geometry_msgs::msg::PoseStamped pregrasp_target;
      pregrasp_target.header.frame_id = planning_frame;
      pregrasp_target.header.stamp = node->now();
      pregrasp_target.pose.position.x = tx;
      pregrasp_target.pose.position.y = ty;
      pregrasp_target.pose.position.z = tz_pre;
      tf2::Quaternion q_desired;
      auto set_identity = [&]() { q_desired.setValue(0.0, 0.0, 0.0, 1.0); };
      auto set_from_current = [&]() {
        tf2::fromMsg(current_pose.orientation, q_desired);
        if (q_desired.length2() > 1e-12) {
          q_desired.normalize();
        } else {
          set_identity();
        }
      };
      auto set_from_rpy = [&](double roll, double pitch, double yaw) { q_desired.setRPY(roll, pitch, yaw); };

      if (!pregrasp_orientation_mode.empty()) {
        if (pregrasp_orientation_mode == "identity") {
          set_identity();
        } else if (pregrasp_orientation_mode == "current") {
          set_from_current();
        } else if (pregrasp_orientation_mode == "current_tool_yaw") {
          set_from_current();
          tf2::Quaternion q_delta;
          q_delta.setRPY(0.0, 0.0, pregrasp_yaw_offset);
          q_desired = q_desired * q_delta;
        } else if (pregrasp_orientation_mode == "current_yaw") {
          set_from_current();
          double r = 0.0, p = 0.0, y = 0.0;
          tf2::Matrix3x3(q_desired).getRPY(r, p, y);
          set_from_rpy(r, p, y + pregrasp_yaw_offset);
        } else if (pregrasp_orientation_mode == "fixed_rpy") {
          const double r = pregrasp_fixed_rpy.size() > 0 ? pregrasp_fixed_rpy[0] : 0.0;
          const double p = pregrasp_fixed_rpy.size() > 1 ? pregrasp_fixed_rpy[1] : 0.0;
          const double y = pregrasp_fixed_rpy.size() > 2 ? pregrasp_fixed_rpy[2] : 0.0;
          set_from_rpy(r, p, y + pregrasp_yaw_offset);
        } else if (pregrasp_orientation_mode == "target_yaw") {
          tf2::Quaternion q_target;
          tf2::fromMsg(target_in_planning.pose.orientation, q_target);
          if (q_target.length2() > 1e-12) {
            q_target.normalize();
          } else {
            q_target.setValue(0.0, 0.0, 0.0, 1.0);
          }
          double r_target = 0.0, p_target = 0.0, y_target = 0.0;
          tf2::Matrix3x3(q_target).getRPY(r_target, p_target, y_target);
          const double r = pregrasp_fixed_rpy.size() > 0 ? pregrasp_fixed_rpy[0] : 0.0;
          const double p = pregrasp_fixed_rpy.size() > 1 ? pregrasp_fixed_rpy[1] : 0.0;
          const double y = pregrasp_fixed_rpy.size() > 2 ? pregrasp_fixed_rpy[2] : 0.0;
          set_from_rpy(r, p, y + y_target + pregrasp_yaw_offset);
        } else {
          set_from_current();
        }
      } else {
        if (pregrasp_use_identity_orientation) {
          set_identity();
        } else {
          set_from_current();
        }
      }
      if (q_desired.length2() > 1e-12) {
        q_desired.normalize();
      } else {
        set_identity();
      }
      pregrasp_target.pose.orientation = tf2::toMsg(q_desired);
      const auto & tq = pregrasp_target.pose.orientation;
      RCLCPP_INFO(
        node->get_logger(),
        "Pregrasp quat=(%.4f %.4f %.4f %.4f) tol_pos=%.3f tol_ori=%.3f",
        static_cast<double>(tq.x),
        static_cast<double>(tq.y),
        static_cast<double>(tq.z),
        static_cast<double>(tq.w),
        goal_pos_tolerance,
        goal_ori_tolerance);
      const bool want_elbow_bias = elbow_constraint_enable && (std::fabs(elbow_delta) > 1e-12 || !std::isnan(elbow_target));
      const bool want_posture_bias = posture_constraint_enable && !posture_joint_names.empty();
      bool used_joint_goal = false;
      if (ik_bias_enable && (want_elbow_bias || want_posture_bias)) {
        const auto * jmg = robot_model->getJointModelGroup(arm_group);
        if (jmg) {
          moveit::core::RobotState preferred(start_state);
          if (want_elbow_bias) {
            double desired = elbow_target;
            if (std::isnan(desired)) {
              desired = start_state.getVariablePosition(elbow_joint_name) + elbow_delta;
            }
            preferred.setVariablePosition(elbow_joint_name, desired);
          }
          if (want_posture_bias) {
            for (size_t i = 0; i < posture_joint_names.size(); ++i) {
              const auto & name = posture_joint_names[i];
              if (name.empty()) {
                continue;
              }
              const double delta = i < posture_joint_deltas.size() ? posture_joint_deltas[i] : 0.0;
              const double desired = start_state.getVariablePosition(name) + delta;
              preferred.setVariablePosition(name, desired);
            }
          }
          preferred.update();

          std::mt19937 rng(static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count()));
          std::uniform_real_distribution<double> unif(-1.0, 1.0);

          auto score_solution = [&](const moveit::core::RobotState & s) -> double {
            double cost = 0.0;
            if (want_elbow_bias) {
              const double d = s.getVariablePosition(elbow_joint_name) - preferred.getVariablePosition(elbow_joint_name);
              cost += d * d;
            }
            if (want_posture_bias) {
              for (const auto & name : posture_joint_names) {
                if (name.empty()) {
                  continue;
                }
                const double d = s.getVariablePosition(name) - preferred.getVariablePosition(name);
                cost += d * d;
              }
            }
            return cost;
          };

          bool found = false;
          double best_cost = std::numeric_limits<double>::infinity();
          std::vector<double> best_joint_values;
          const int tries = std::max(1, ik_bias_attempts);
          const double noise = std::max(0.0, ik_bias_noise);
          const double timeout = std::max(0.01, ik_bias_timeout);

          for (int attempt = 0; attempt < tries; ++attempt) {
            moveit::core::RobotState ik_state(preferred);
            if (noise > 0.0) {
              for (const auto & name : jmg->getVariableNames()) {
                const auto b = robot_model->getVariableBounds(name);
                if (b.position_bounded_) {
                  continue;
                }
                const double cur = ik_state.getVariablePosition(name);
                ik_state.setVariablePosition(name, cur + noise * unif(rng));
              }
            }
            ik_state.update();
            const bool ik_ok = ik_state.setFromIK(jmg, pregrasp_target.pose, eef_link, timeout);
            if (!ik_ok) {
              continue;
            }
            const double cost = score_solution(ik_state);
            if (cost < best_cost) {
              best_cost = cost;
              best_joint_values.clear();
              ik_state.copyJointGroupPositions(jmg, best_joint_values);
              found = true;
            }
          }

          if (found && !best_joint_values.empty()) {
            const auto & var_names = jmg->getVariableNames();
            constexpr double kTwoPi = 6.283185307179586;
            auto wrap_near = [&](double current, double target) {
              const double delta = std::remainder(target - current, kTwoPi);
              return current + delta;
            };
            for (const auto & name : wrap_joints) {
              if (name.empty()) {
                continue;
              }
              const auto bounds = robot_model->getVariableBounds(name);
              const bool can_wrap = !bounds.position_bounded_ || (bounds.max_position_ - bounds.min_position_) > 6.0;
              if (!can_wrap) {
                continue;
              }
              for (size_t i = 0; i < var_names.size() && i < best_joint_values.size(); ++i) {
                if (var_names[i] != name) {
                  continue;
                }
                const double cur = start_state.getVariablePosition(name);
                best_joint_values[i] = wrap_near(cur, best_joint_values[i]);
                break;
              }
            }
            arm.setStartStateToCurrentState();
            arm.clearPoseTargets();
            arm.setJointValueTarget(best_joint_values);
            used_joint_goal = true;
          }
        }
      }

      if (!used_joint_goal) {
        arm.setPoseTarget(pregrasp_target);
      }
      moveit::planning_interface::MoveGroupInterface::Plan to_pregrasp_plan;
      moveit::core::MoveItErrorCode pregrasp_ret = moveit::core::MoveItErrorCode::FAILURE;
      const bool have_best_pre = plan_best_of_n(
        node, arm, best_of_plans, best_plan_metric, wrap_joints, to_pregrasp_plan, pregrasp_ret, best_of_time_budget_sec);
      if (!have_best_pre) {
        pregrasp_ret = moveit::core::MoveItErrorCode::FAILURE;
      }
      if (pregrasp_ret != moveit::core::MoveItErrorCode::SUCCESS && pregrasp_fallback_enable) {
        const double tol = std::max(goal_ori_tolerance, pregrasp_fallback_ori_tolerance);
        if (tol > 0.0) {
          arm.setGoalOrientationTolerance(tol);
        }
        arm.setPlanningTime(std::max(0.1, std::max(planning_time, pregrasp_fallback_planning_time)));
        arm.setNumPlanningAttempts(std::max(1, std::max(planning_attempts, pregrasp_fallback_planning_attempts)));

        arm.setStartStateToCurrentState();
        arm.clearPoseTargets();
        arm.setPoseTarget(pregrasp_target);
        if (!plan_best_of_n(
              node,
              arm,
              best_of_plans,
              best_plan_metric,
              wrap_joints,
              to_pregrasp_plan,
              pregrasp_ret,
              best_of_time_budget_sec)) {
          pregrasp_ret = moveit::core::MoveItErrorCode::FAILURE;
        }

        if (pregrasp_ret != moveit::core::MoveItErrorCode::SUCCESS && pregrasp_fallback_position_only) {
          arm.setStartStateToCurrentState();
          arm.clearPoseTargets();
          arm.setPositionTarget(tx, ty, tz_pre, eef_link);
          if (!plan_best_of_n(
                node,
                arm,
                best_of_plans,
                best_plan_metric,
                wrap_joints,
                to_pregrasp_plan,
                pregrasp_ret,
                best_of_time_budget_sec)) {
            pregrasp_ret = moveit::core::MoveItErrorCode::FAILURE;
          }
        }

        if (goal_ori_tolerance > 0.0) {
          arm.setGoalOrientationTolerance(goal_ori_tolerance);
        }
        arm.setPlanningTime(std::max(0.1, planning_time));
        arm.setNumPlanningAttempts(std::max(1, planning_attempts));
      }
      if (pregrasp_ret != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Plan to pregrasp failed: ret=%d goal_xyz=(%.3f %.3f %.3f) pregrasp_z_offset=%.3f planning_time=%.2f attempts=%d",
          pregrasp_ret.val,
          tx,
          ty,
          tz_pre,
          pregrasp_z_offset,
          planning_time,
          planning_attempts);
        finish_fail("pregrasp_plan_failed");
        return;
      }
      if (plan_only) {
        publish_grasp_state("planned", "pregrasp_plan_ready");
        RCLCPP_INFO(node->get_logger(), "Plan-only check passed; no robot motion was executed.");
        if (execute_once) {
          done.store(true);
        }
        executing.store(false);
        return;
      }
      const auto & pregrasp_joint_trajectory = to_pregrasp_plan.trajectory_.joint_trajectory;
      if (pregrasp_joint_trajectory.joint_names.empty() || pregrasp_joint_trajectory.points.empty()) {
        finish_fail("pregrasp_trajectory_empty");
        return;
      }
      moveit::core::RobotState pregrasp_state = start_state;
      const auto & pregrasp_endpoint = pregrasp_joint_trajectory.points.back().positions;
      if (pregrasp_endpoint.size() != pregrasp_joint_trajectory.joint_names.size()) {
        finish_fail("pregrasp_trajectory_invalid");
        return;
      }
      for (size_t i = 0; i < pregrasp_endpoint.size(); ++i) {
        const auto * joint = robot_model->getJointModel(pregrasp_joint_trajectory.joint_names[i]);
        if (joint && joint->getVariableCount() == 1) {
          const double position = pregrasp_endpoint[i];
          pregrasp_state.setJointPositions(joint, &position);
        }
      }
      pregrasp_state.update();
      geometry_msgs::msg::Pose pregrasp_pose =
        tf2::toMsg(pregrasp_state.getGlobalLinkTransform(eef_link));

      if (pregrasp_orientation_enforce && !single_descend_enable && !inspect_after_pregrasp && !inspect_after_grasp) {
        tf2::Quaternion q_cur;
        tf2::fromMsg(pregrasp_pose.orientation, q_cur);
        if (q_cur.length2() > 1e-12) {
          q_cur.normalize();
        } else {
          q_cur.setValue(0.0, 0.0, 0.0, 1.0);
        }
        tf2::Quaternion q_des;
        tf2::fromMsg(pregrasp_target.pose.orientation, q_des);
        if (q_des.length2() > 1e-12) {
          q_des.normalize();
        } else {
          q_des.setValue(0.0, 0.0, 0.0, 1.0);
        }

        const double dot = std::fabs(q_cur.x() * q_des.x() + q_cur.y() * q_des.y() + q_cur.z() * q_des.z() + q_cur.w() * q_des.w());
        const double clamped_dot = std::min(1.0, std::max(0.0, dot));
        const double angle = 2.0 * std::acos(clamped_dot);

        if (angle >= std::max(0.0, pregrasp_orientation_enforce_min_angle)) {
          RCLCPP_INFO(
            node->get_logger(),
            "Pregrasp orientation enforce: angle=%.3f rad",
            angle);

          const double old_time = planning_time;
          const int old_attempts = planning_attempts;
          const double old_tol_ori = goal_ori_tolerance;

          arm.setStartStateToCurrentState();
          arm.clearPoseTargets();
          geometry_msgs::msg::PoseStamped enforce_pose;
          enforce_pose.header.frame_id = planning_frame;
          enforce_pose.header.stamp = node->now();
          enforce_pose.pose = pregrasp_pose;
          enforce_pose.pose.orientation = pregrasp_target.pose.orientation;
          arm.setPoseTarget(enforce_pose, eef_link);
          arm.setPlanningTime(std::max(0.1, pregrasp_orientation_enforce_planning_time));
          arm.setNumPlanningAttempts(std::max(1, pregrasp_orientation_enforce_attempts));
          arm.setGoalOrientationTolerance(std::max(0.001, pregrasp_orientation_enforce_ori_tolerance));

          moveit::planning_interface::MoveGroupInterface::Plan enforce_plan;
          const auto enforce_ret = arm.plan(enforce_plan);
          if (enforce_ret == moveit::core::MoveItErrorCode::SUCCESS) {
            if (!execute_plan(node, arm, enforce_plan)) {
              finish_fail();
              return;
            }
            arm.setStartStateToCurrentState();
            pregrasp_pose = arm.getCurrentPose(eef_link).pose;
          }

          arm.setPlanningTime(std::max(0.1, old_time));
          arm.setNumPlanningAttempts(std::max(1, old_attempts));
          if (old_tol_ori > 0.0) {
            arm.setGoalOrientationTolerance(old_tol_ori);
          }
        }
      }

      bool blend_pregrasp_correction_into_descent = false;
      if (!single_descend_enable) {
        blend_pregrasp_correction_into_descent = true;
      }

      auto inspect_geometry = [&](const geometry_msgs::msg::Pose & tcp_pose, const char * stage,
                                  const char * reason) {
        geometry_msgs::msg::PoseStamped inspect_target;
        {
          std::scoped_lock<std::mutex> lock(target_mutex);
          inspect_target = latest_target;
        }
        inspect_target = transform_pose(node, tf_buffer, inspect_target, planning_frame);
        tf2::Transform tcp_tf;
        tf2::Transform target_tf;
        tf2::fromMsg(tcp_pose, tcp_tf);
        tf2::fromMsg(inspect_target.pose, target_tf);
        const auto target_in_tcp = tcp_tf.inverse() * target_tf.getOrigin();
        RCLCPP_INFO(
          node->get_logger(),
          "Inspect %s: tcp_xyz=(%.4f %.4f %.4f) target_xyz=(%.4f %.4f %.4f) "
          "target_in_tcp_xyz=(%.4f %.4f %.4f)",
          stage,
          static_cast<double>(tcp_pose.position.x),
          static_cast<double>(tcp_pose.position.y),
          static_cast<double>(tcp_pose.position.z),
          static_cast<double>(inspect_target.pose.position.x),
          static_cast<double>(inspect_target.pose.position.y),
          static_cast<double>(inspect_target.pose.position.z),
          target_in_tcp.x(), target_in_tcp.y(), target_in_tcp.z());
        RCLCPP_INFO(
          node->get_logger(), "Inspect %s: tcp_quat=(%.6f %.6f %.6f %.6f)", stage,
          static_cast<double>(tcp_pose.orientation.x), static_cast<double>(tcp_pose.orientation.y),
          static_cast<double>(tcp_pose.orientation.z), static_cast<double>(tcp_pose.orientation.w));
        publish_grasp_state("inspect", reason);
      };

      if (inspect_after_pregrasp) {
        inspect_geometry(pregrasp_pose, "pregrasp", "pregrasp_geometry_ready");
        if (execute_once) {
          done.store(true);
        }
        executing.store(false);
        return;
      }

      if (single_descend_enable) {
        arm.setStartStateToCurrentState();
        const auto pose_before_descend = arm.getCurrentPose(eef_link).pose;
        const double current_z = static_cast<double>(pose_before_descend.position.z);
        const double min_safe_z = tz_obj + std::max(0.0, single_descend_min_height_above_target);
        if (current_z <= min_safe_z + 1e-4) {
          RCLCPP_WARN(
            node->get_logger(),
            "Single descend refused: current_z=%.4f is not above safe_z=%.4f (target_z=%.4f, min_height=%.4f)",
            current_z, min_safe_z, tz_obj, single_descend_min_height_above_target);
          finish_fail("single_descend_refused_safe_height");
          return;
        }

        const double distance = std::min(
          std::max(0.0, single_descend_max_distance), current_z - min_safe_z);
        if (distance <= 1e-4) {
          RCLCPP_WARN(node->get_logger(), "Single descend refused: allowed distance is %.4f m", distance);
          finish_fail("single_descend_refused_zero_distance");
          return;
        }

        RCLCPP_INFO(
          node->get_logger(),
          "Single descend: current_z=%.4f target_z=%.4f distance=%.4f safe_z=%.4f",
          current_z, current_z - distance, distance, min_safe_z);
        auto pose_after_descend = pose_before_descend;
        pose_after_descend.position.z = current_z - distance;
        moveit_msgs::msg::RobotTrajectory single_descend_traj;
        const double fraction = arm.computeCartesianPath(
          {pose_before_descend, pose_after_descend}, eef_step, 0.0, single_descend_traj, true);
        if (fraction < 1.0 - 1e-3) {
          RCLCPP_ERROR(
            node->get_logger(), "Single descend refused: Cartesian fraction %.4f is not complete", fraction);
          finish_fail("single_descend_cartesian_incomplete");
          return;
        }

        auto current_state_ptr = arm.getCurrentState(current_state_timeout);
        if (!current_state_ptr || !execute_trajectory(
              node, arm, robot_model, arm_group, *current_state_ptr, single_descend_traj, vel_scale, acc_scale)) {
          finish_fail("single_descend_execute_failed");
          return;
        }
        arm.setStartStateToCurrentState();
        if (single_descend_close_gripper) {
          RCLCPP_ERROR(
            node->get_logger(),
            "Single-descend close is disabled: closing requires the full closed-loop centering check");
          finish_fail("single_descend_close_requires_closed_loop");
          return;
        }
        inspect_geometry(arm.getCurrentPose(eef_link).pose, "single_descend", "single_descend_geometry_ready");
        if (execute_once) {
          done.store(true);
        }
        executing.store(false);
        return;
      }

      geometry_msgs::msg::Pose grasp_pose = pregrasp_pose;
      if (blend_pregrasp_correction_into_descent) {
        grasp_pose.position.x = pregrasp_target.pose.position.x;
        grasp_pose.position.y = pregrasp_target.pose.position.y;
      }
      const double desired_grasp_z_uncapped =
        tz_obj + grasp_z_offset - finger_tip_z_offset - std::max(0.0, descend_below_target);
      double grasp_z_clamped = std::max(desired_grasp_z_uncapped, min_eef_z);
      if (min_grasp_height_above_target > 0.0) {
        grasp_z_clamped = std::max(grasp_z_clamped, tz_obj + min_grasp_height_above_target);
      }
      grasp_pose.position.z = grasp_z_clamped;
      if (std::fabs(grasp_z_clamped - desired_grasp_z_uncapped) > 1e-4) {
        RCLCPP_WARN(
          node->get_logger(),
          "Grasp Z clamped: desired=%.4f clamped=%.4f (min_eef_z=%.4f)",
          desired_grasp_z_uncapped,
          grasp_z_clamped,
          min_eef_z);
      }
      RCLCPP_INFO(
        node->get_logger(),
        "Descent plan: pregrasp_z=%.3f grasp_z=%.4f descent=%.4f tz_obj=%.4f "
        "(grasp_z_offset=%.3f finger_tip=%.3f below_target=%.3f)",
        static_cast<double>(pregrasp_pose.position.z),
        grasp_z_clamped,
        std::max(0.0, static_cast<double>(pregrasp_pose.position.z) - grasp_z_clamped),
        tz_obj,
        grasp_z_offset,
        finger_tip_z_offset,
        descend_below_target);

      geometry_msgs::msg::Pose lift_pose = pregrasp_pose;
      const double lift_delta = std::max(0.0, lift_z_offset - pregrasp_z_offset);
      lift_pose.position.z = static_cast<double>(pregrasp_pose.position.z) + lift_delta;

      std::vector<geometry_msgs::msg::Pose> down_waypoints;
      publish_grasp_state("contact", "descending_to_grasp");
      down_waypoints.push_back(pregrasp_pose);
      down_waypoints.push_back(grasp_pose);

      moveit_msgs::msg::RobotTrajectory down_traj;
      arm.setStartState(pregrasp_state);
      const double down_fraction = arm.computeCartesianPath(
        {grasp_pose}, eef_step, 0.0, down_traj, descend_avoid_collisions);
      if (down_fraction < 1.0 - 1e-3) {
        RCLCPP_WARN(
          node->get_logger(),
          "Down Cartesian path partial: fraction=%.2f (will supplement with Z descent)",
          down_fraction);
      }
      if (down_fraction < min_fraction) {
        RCLCPP_ERROR(node->get_logger(), "Down Cartesian fraction too low for merged trajectory: %.2f", down_fraction);
        finish_fail("merged_down_cartesian_incomplete");
        return;
      } else {
        moveit_msgs::msg::RobotTrajectory merged_traj = to_pregrasp_plan.trajectory_;
        auto & merged_joint_trajectory = merged_traj.joint_trajectory;
        const auto & down_joint_trajectory = down_traj.joint_trajectory;
        if (merged_joint_trajectory.joint_names != down_joint_trajectory.joint_names) {
          finish_fail("merged_trajectory_joint_mismatch");
          return;
        }
        for (size_t i = 1; i < down_joint_trajectory.points.size(); ++i) {
          merged_joint_trajectory.points.push_back(down_joint_trajectory.points[i]);
        }
        RCLCPP_INFO(node->get_logger(), "Executing merged pregrasp-to-grasp trajectory");
        if (!execute_trajectory(
              node, arm, robot_model, arm_group, start_state, merged_traj, vel_scale, acc_scale)) {
          finish_fail();
          return;
        }
      }

      arm.setStartStateToCurrentState();

      const double desired_grasp_z_final = static_cast<double>(grasp_pose.position.z);
      arm.setStartStateToCurrentState();
      const auto pose_after_main_down = arm.getCurrentPose(eef_link).pose;
      const double gap_after_main =
        static_cast<double>(pose_after_main_down.position.z) - desired_grasp_z_final;
      if (gap_after_main > std::max(0.0, reach_grasp_tolerance)) {
        if (!descend_eef_to_z(
              node,
              arm,
              robot_model,
              arm_group,
              eef_link,
              desired_grasp_z_final,
              min_eef_z,
              eef_step,
              min_fraction,
              descend_avoid_collisions,
              vel_scale,
              acc_scale,
              current_state_timeout,
              "Reach grasp height")) {
          finish_fail("reach_grasp_height_failed");
          return;
        }
      } else {
        RCLCPP_INFO(
          node->get_logger(),
          "Grasp height OK after main descent: z=%.4f target=%.4f gap=%.4f",
          static_cast<double>(pose_after_main_down.position.z),
          desired_grasp_z_final,
          gap_after_main);
      }

      if (auto_final_descend_enable) {
        arm.setStartStateToCurrentState();
        const auto pose_after_down_pre = arm.getCurrentPose(eef_link).pose;
        const double gap_to_grasp_target =
          static_cast<double>(pose_after_down_pre.position.z) - desired_grasp_z_final;
        const double margin = std::max(0.0, auto_final_descend_margin);
        const double max_extra = std::max(0.0, auto_final_descend_max);
        const double extra = std::min(std::max(0.0, gap_to_grasp_target - margin), max_extra);
        if (extra > 1e-4) {
          const double auto_target_z = std::max(
            desired_grasp_z_final,
            static_cast<double>(pose_after_down_pre.position.z) - extra);
          if (!descend_eef_to_z(
                node,
                arm,
                robot_model,
                arm_group,
                eef_link,
                auto_target_z,
                min_eef_z,
                eef_step,
                min_fraction,
                descend_avoid_collisions,
                vel_scale,
                acc_scale,
                current_state_timeout,
                "Auto final descend")) {
            finish_fail("auto_final_descend_failed");
            return;
          }
        }
      }

      if (force_extra_descend > 1e-6) {
        arm.setStartStateToCurrentState();
        const auto pose_before_force = arm.getCurrentPose(eef_link).pose;
        const double current_z = static_cast<double>(pose_before_force.position.z);
        if (current_z > desired_grasp_z_final + reach_grasp_tolerance) {
          const double force_target_z = std::max(
            desired_grasp_z_final,
            current_z - std::max(0.0, force_extra_descend));
          if (!descend_eef_to_z(
                node,
                arm,
                robot_model,
                arm_group,
                eef_link,
                force_target_z,
                min_eef_z,
                eef_step,
                min_fraction,
                descend_avoid_collisions,
                vel_scale,
                acc_scale,
                current_state_timeout,
                "Force extra descend")) {
            finish_fail("force_extra_descend_failed");
            return;
          }
        }
      }

      arm.setStartStateToCurrentState();
      {
        const auto pose_before_grasp = arm.getCurrentPose(eef_link).pose;
        const double gap = static_cast<double>(pose_before_grasp.position.z) - desired_grasp_z_final;
        if (gap > std::max(0.0, reach_grasp_tolerance)) {
          RCLCPP_INFO(
            node->get_logger(),
            "Still %.4f m above grasp target (tol=%.4f), retrying descent",
            gap,
            reach_grasp_tolerance);
          if (!descend_eef_to_z(
                node,
                arm,
                robot_model,
                arm_group,
                eef_link,
                desired_grasp_z_final,
                min_eef_z,
                eef_step,
                min_fraction,
                descend_avoid_collisions,
                vel_scale,
                acc_scale,
                current_state_timeout,
                "Ensure grasp reach")) {
            finish_fail("ensure_grasp_reach_failed");
            return;
          }
        } else if (gap < -0.02) {
          RCLCPP_WARN(
            node->get_logger(),
            "Grasp height %.4f m below target %.4f (over-descended by %.4f m)",
            static_cast<double>(pose_before_grasp.position.z),
            desired_grasp_z_final,
            -gap);
          finish_fail("grasp_over_descended");
          return;
        }
      }

      arm.setStartStateToCurrentState();

      if (inspect_after_grasp) {
        inspect_geometry(arm.getCurrentPose(eef_link).pose, "grasp", "grasp_geometry_ready");
        if (execute_once) {
          done.store(true);
        }
        executing.store(false);
        return;
      }

      if (pre_gripper_close_pause_ms > 0) {
        RCLCPP_INFO(
          node->get_logger(), "Pause %d ms at grasp height before closing gripper", pre_gripper_close_pause_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(pre_gripper_close_pause_ms));
        arm.setStartStateToCurrentState();
      }

      if (use_direct_gripper_action) {
        const auto per_goal_timeout = std::chrono::milliseconds(std::max(1000, gripper_action_timeout_ms));
        if (gripper_slow_close) {
          RCLCPP_INFO(
            node->get_logger(),
            "Slow gripper close: steps=%d delay_ms=%d (total ~%d ms)",
            gripper_close_steps,
            gripper_close_step_delay_ms,
            gripper_close_steps * gripper_close_step_delay_ms);
          if (!send_gripper_action_slow_close(
                node,
                gripper_client,
                gripper_open_pos,
                gripper_close_pos,
                gripper_max_effort,
                gripper_close_steps,
                per_goal_timeout,
                std::chrono::milliseconds(std::max(0, gripper_close_step_delay_ms)),
                gripper_wait_result,
                gripper_timeout_is_success)) {
            finish_fail();
            return;
          }
        } else if (!send_gripper_action(
                     node,
                     gripper_client,
                     gripper_close_pos,
                     gripper_max_effort,
                     per_goal_timeout,
                     gripper_wait_result,
                     gripper_timeout_is_success)) {
          finish_fail();
          return;
        }
      } else {
        gripper->setStartState(start_state);
        gripper->setNamedTarget(gripper_close_named_target);
        moveit::planning_interface::MoveGroupInterface::Plan gripper_close_plan;
        if (gripper->plan(gripper_close_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "Plan gripper close failed");
          finish_fail();
          return;
        }
        if (!execute_plan(node, *gripper, gripper_close_plan)) {
          finish_fail();
          return;
        }
      }

      publish_grasp_state("grasped", "gripper_closed");
      if (!lift_after_close_enable) {
        RCLCPP_INFO(node->get_logger(), "Gripper closed; holding TCP position (lift disabled)");
        if (post_gripper_close_settle_ms > 0) {
          std::this_thread::sleep_for(std::chrono::milliseconds(post_gripper_close_settle_ms));
        }
        inspect_geometry(arm.getCurrentPose(eef_link).pose, "close", "gripper_contact_ready");
        if (execute_once) {
          done.store(true);
        }
        executing.store(false);
        return;
      }

      arm.setStartStateToCurrentState();
      const auto pose_after_down = arm.getCurrentPose(eef_link).pose;
      geometry_msgs::msg::Pose grasp_pose_actual = pose_after_down;
      geometry_msgs::msg::Pose lift_pose_actual = pose_after_down;
      lift_pose_actual.position.z = static_cast<double>(pose_after_down.position.z) + lift_delta;
      RCLCPP_INFO(
        node->get_logger(),
        "After down: current_z=%.3f target_z=%.3f (delta_current_to_target=%.3f m)",
        static_cast<double>(pose_after_down.position.z),
        tz_obj,
        static_cast<double>(pose_after_down.position.z) - tz_obj);

      if (post_gripper_close_settle_ms > 0) {
        RCLCPP_INFO(
          node->get_logger(), "Settling %d ms after gripper close before lift", post_gripper_close_settle_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(post_gripper_close_settle_ms));
        arm.setStartStateToCurrentState();
        grasp_pose_actual = arm.getCurrentPose(eef_link).pose;
        lift_pose_actual = grasp_pose_actual;
        lift_pose_actual.position.z = static_cast<double>(grasp_pose_actual.position.z) + lift_delta;
      }

      if (inspect_after_close) {
        inspect_geometry(grasp_pose_actual, "close", "gripper_contact_ready");
        if (execute_once) {
          done.store(true);
        }
        executing.store(false);
        return;
      }

      double remaining_lift_delta = lift_delta;
      if (staged_lift_enable && lift_delta > 1e-6) {
        const double step = std::max(0.0, staged_lift_first_step);
        if (step > 1e-6 && step < lift_delta - 1e-6) {
          geometry_msgs::msg::Pose mid_pose = grasp_pose_actual;
          mid_pose.position.z = static_cast<double>(grasp_pose_actual.position.z) + step;

          std::vector<geometry_msgs::msg::Pose> stage1_waypoints;
          stage1_waypoints.push_back(grasp_pose_actual);
          stage1_waypoints.push_back(mid_pose);

          moveit_msgs::msg::RobotTrajectory stage1_traj;
          arm.setStartStateToCurrentState();
          const double stage1_fraction =
            arm.computeCartesianPath(stage1_waypoints, eef_step, 0.0, stage1_traj, avoid_collisions);
          if (stage1_fraction >= min_fraction) {
            auto current_state_ptr = arm.getCurrentState(current_state_timeout);
            if (!current_state_ptr) {
              finish_fail();
              return;
            }
            if (!execute_trajectory(
              node, arm, robot_model, arm_group, *current_state_ptr, stage1_traj, lift_vel_scale, lift_acc_scale))
            {
              finish_fail();
              return;
            }
            remaining_lift_delta = lift_delta - step;
            if (staged_lift_pause_ms > 0) {
              RCLCPP_INFO(node->get_logger(), "Pause %d ms after staged lift step 1", staged_lift_pause_ms);
              std::this_thread::sleep_for(std::chrono::milliseconds(staged_lift_pause_ms));
            }
            arm.setStartStateToCurrentState();
            const auto pose_after_stage1 = arm.getCurrentPose(eef_link).pose;
            grasp_pose_actual = pose_after_stage1;
            lift_pose_actual = pose_after_stage1;
            lift_pose_actual.position.z = static_cast<double>(pose_after_stage1.position.z) + (lift_delta - step);
          }
        }
      }

      arm.setStartStateToCurrentState();
      grasp_pose_actual = arm.getCurrentPose(eef_link).pose;
      lift_pose_actual = grasp_pose_actual;
      lift_pose_actual.position.z =
        static_cast<double>(grasp_pose_actual.position.z) + remaining_lift_delta;

      std::vector<geometry_msgs::msg::Pose> up_waypoints;
      up_waypoints.push_back(grasp_pose_actual);
      up_waypoints.push_back(lift_pose_actual);

      moveit_msgs::msg::RobotTrajectory up_traj;
      arm.setStartStateToCurrentState();
      const double up_fraction = arm.computeCartesianPath(up_waypoints, eef_step, 0.0, up_traj, avoid_collisions);
      if (up_fraction < min_fraction) {
        RCLCPP_ERROR(node->get_logger(), "Up Cartesian fraction too low: %.2f", up_fraction);
        arm.setStartStateToCurrentState();
        arm.clearPoseTargets();
        geometry_msgs::msg::Pose lift_pose_for_pose_plan = lift_pose_actual;
        lift_pose_for_pose_plan.orientation = pregrasp_target.pose.orientation;
        arm.setPoseTarget(lift_pose_for_pose_plan);
        moveit::planning_interface::MoveGroupInterface::Plan to_lift_plan;
        auto lift_ret = arm.plan(to_lift_plan);
        if (lift_ret != moveit::core::MoveItErrorCode::SUCCESS) {
          arm.setStartStateToCurrentState();
          arm.clearPoseTargets();
          arm.setPositionTarget(
            static_cast<double>(lift_pose_actual.position.x),
            static_cast<double>(lift_pose_actual.position.y),
            static_cast<double>(lift_pose_actual.position.z),
            eef_link);
          lift_ret = arm.plan(to_lift_plan);
          if (lift_ret != moveit::core::MoveItErrorCode::SUCCESS) {
            finish_fail();
            return;
          }
        }
        if (!execute_plan(node, arm, to_lift_plan)) {
          finish_fail();
          return;
        }
      } else {
        auto current_state_ptr = arm.getCurrentState(current_state_timeout);
        if (!current_state_ptr) {
          finish_fail();
          return;
        }
        if (!execute_trajectory(node, arm, robot_model, arm_group, *current_state_ptr, up_traj, lift_vel_scale, lift_acc_scale)) {
          finish_fail();
          return;
        }
      }

      publish_grasp_state("lifted", "lift_complete");

      if (execute_once) {
        done.store(true);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(), "Pick failed: %s", e.what());
      publish_grasp_state("failed", "exception");
      if (execute_once) {
        done.store(true);
      }
    }

    executing.store(false);
  };

  auto timer = node->create_wall_timer(std::chrono::milliseconds(200), tick, main_cb_group);
  auto exit_timer = node->create_wall_timer(std::chrono::milliseconds(100), [&]() {
    if (done.load()) {
      executor.cancel();
    }
  });

  executor.spin();
  rclcpp::shutdown();
  (void)timer;
  (void)exit_timer;
  (void)sub;
  (void)joint_sub;
  return 0;
}
