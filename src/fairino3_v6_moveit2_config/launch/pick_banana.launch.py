from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "fairino3_v6_robot", package_name="fairino3_v6_moveit2_config"
    ).to_moveit_configs()

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    start_move_group_arg = DeclareLaunchArgument("start_move_group", default_value="true")

    target_pose_topic_arg = DeclareLaunchArgument("target_pose_topic", default_value="/yolo/target_pose")
    grasp_state_topic_arg = DeclareLaunchArgument("grasp_state_topic", default_value="/world_model/grasp_state")
    target_object_id_arg = DeclareLaunchArgument("target_object_id", default_value="banana-1")
    execute_once_arg = DeclareLaunchArgument("execute_once", default_value="true")
    plan_only_arg = DeclareLaunchArgument("plan_only", default_value="false")
    inspect_after_pregrasp_arg = DeclareLaunchArgument("inspect_after_pregrasp", default_value="false")
    inspect_after_grasp_arg = DeclareLaunchArgument("inspect_after_grasp", default_value="false")
    inspect_after_close_arg = DeclareLaunchArgument("inspect_after_close", default_value="false")

    best_of_plans_arg = DeclareLaunchArgument("best_of_plans", default_value="15")
    best_of_time_budget_sec_arg = DeclareLaunchArgument("best_of_time_budget_sec", default_value="6.0")
    best_plan_metric_arg = DeclareLaunchArgument("best_plan_metric", default_value="joint_l1")

    refine_at_pregrasp_enable_arg = DeclareLaunchArgument("refine_at_pregrasp_enable", default_value="true")
    refine_at_pregrasp_cartesian_arg = DeclareLaunchArgument("refine_at_pregrasp_cartesian", default_value="true")
    refine_at_pregrasp_min_xy_arg = DeclareLaunchArgument("refine_at_pregrasp_min_xy", default_value="0.004")
    closed_loop_max_iterations_arg = DeclareLaunchArgument("closed_loop_max_iterations", default_value="5")
    closed_loop_xyz_tolerance_arg = DeclareLaunchArgument("closed_loop_xyz_tolerance", default_value="0.007")
    closed_loop_orientation_tolerance_arg = DeclareLaunchArgument(
        "closed_loop_orientation_tolerance", default_value="0.05"
    )
    closed_loop_max_translation_step_arg = DeclareLaunchArgument(
        "closed_loop_max_translation_step", default_value="0.01"
    )
    closed_loop_max_orientation_step_arg = DeclareLaunchArgument(
        "closed_loop_max_orientation_step", default_value="0.05"
    )

    add_table_collision_arg = DeclareLaunchArgument("add_table_collision", default_value="false")
    avoid_collisions_arg = DeclareLaunchArgument("avoid_collisions", default_value="false")

    target_x_offset_arg = DeclareLaunchArgument("target_x_offset", default_value="0.0")
    target_y_offset_arg = DeclareLaunchArgument("target_y_offset", default_value="0.0")
    target_z_offset_arg = DeclareLaunchArgument("target_z_offset", default_value="0.02")
    target_z_override_arg = DeclareLaunchArgument("target_z_override", default_value="nan")

    pregrasp_z_offset_arg = DeclareLaunchArgument("pregrasp_z_offset", default_value="0.08")
    grasp_z_offset_arg = DeclareLaunchArgument("grasp_z_offset", default_value="0.010")
    finger_tip_z_offset_arg = DeclareLaunchArgument("finger_tip_z_offset", default_value="0.010")
    lift_z_offset_arg = DeclareLaunchArgument("lift_z_offset", default_value="0.18")
    lift_only_arg = DeclareLaunchArgument("lift_only", default_value="false")
    lift_only_distance_arg = DeclareLaunchArgument("lift_only_distance", default_value="0.02")

    eef_step_arg = DeclareLaunchArgument("eef_step", default_value="0.01")
    min_fraction_arg = DeclareLaunchArgument("min_fraction", default_value="0.5")

    min_grasp_height_above_target_arg = DeclareLaunchArgument("min_grasp_height_above_target", default_value="0.0")
    auto_final_descend_enable_arg = DeclareLaunchArgument("auto_final_descend_enable", default_value="false")
    auto_final_descend_margin_arg = DeclareLaunchArgument("auto_final_descend_margin", default_value="0.001")
    auto_final_descend_max_arg = DeclareLaunchArgument("auto_final_descend_max", default_value="0.025")
    force_extra_descend_arg = DeclareLaunchArgument("force_extra_descend", default_value="0.0")
    min_eef_z_arg = DeclareLaunchArgument("min_eef_z", default_value="-0.08")
    descend_below_target_arg = DeclareLaunchArgument("descend_below_target", default_value="0.0")
    descend_avoid_collisions_arg = DeclareLaunchArgument("descend_avoid_collisions", default_value="false")
    reach_grasp_tolerance_arg = DeclareLaunchArgument("reach_grasp_tolerance", default_value="0.05")
    single_descend_enable_arg = DeclareLaunchArgument("single_descend_enable", default_value="false")
    single_descend_close_gripper_arg = DeclareLaunchArgument(
        "single_descend_close_gripper", default_value="false"
    )
    single_descend_max_distance_arg = DeclareLaunchArgument("single_descend_max_distance", default_value="0.01")
    single_descend_min_height_above_target_arg = DeclareLaunchArgument(
        "single_descend_min_height_above_target", default_value="0.08"
    )

    planning_time_arg = DeclareLaunchArgument("planning_time", default_value="3.0")
    planning_attempts_arg = DeclareLaunchArgument("planning_attempts", default_value="5")
    planning_pipeline_id_arg = DeclareLaunchArgument("planning_pipeline_id", default_value="ompl")

    vel_scale_arg = DeclareLaunchArgument("vel_scale", default_value="0.3")
    acc_scale_arg = DeclareLaunchArgument("acc_scale", default_value="0.3")
    lift_vel_scale_arg = DeclareLaunchArgument("lift_vel_scale", default_value="0.05")
    lift_acc_scale_arg = DeclareLaunchArgument("lift_acc_scale", default_value="0.05")

    goal_pos_tolerance_arg = DeclareLaunchArgument("goal_pos_tolerance", default_value="0.01")
    goal_ori_tolerance_arg = DeclareLaunchArgument("goal_ori_tolerance", default_value="0.05")

    gripper_action_name_arg = DeclareLaunchArgument(
        "gripper_action_name", default_value="/robotiq_gripper_controller/gripper_cmd"
    )
    gripper_open_pos_arg = DeclareLaunchArgument("gripper_open_pos", default_value="0.0")
    gripper_close_pos_arg = DeclareLaunchArgument("gripper_close_pos", default_value="0.75")
    gripper_max_effort_arg = DeclareLaunchArgument("gripper_max_effort", default_value="100.0")
    gripper_action_timeout_ms_arg = DeclareLaunchArgument("gripper_action_timeout_ms", default_value="15000")

    pregrasp_orientation_mode_arg = DeclareLaunchArgument("pregrasp_orientation_mode", default_value="target_yaw")
    pregrasp_fixed_rpy_arg = DeclareLaunchArgument("pregrasp_fixed_rpy", default_value="[3.14159, 0.0, 0.0]")
    pregrasp_yaw_offset_arg = DeclareLaunchArgument("pregrasp_yaw_offset", default_value="1.57079632679")

    current_state_timeout_arg = DeclareLaunchArgument("current_state_timeout", default_value="5.0")
    min_target_age_sec_arg = DeclareLaunchArgument("min_target_age_sec", default_value="0.2")
    target_pos_epsilon_arg = DeclareLaunchArgument("target_pos_epsilon", default_value="0.01")
    retry_interval_sec_arg = DeclareLaunchArgument("retry_interval_sec", default_value="1.0")

    gripper_slow_close_arg = DeclareLaunchArgument("gripper_slow_close", default_value="false")
    gripper_close_steps_arg = DeclareLaunchArgument("gripper_close_steps", default_value="12")
    gripper_close_step_delay_ms_arg = DeclareLaunchArgument("gripper_close_step_delay_ms", default_value="350")
    gripper_wait_result_arg = DeclareLaunchArgument("gripper_wait_result", default_value="true")
    gripper_timeout_is_success_arg = DeclareLaunchArgument("gripper_timeout_is_success", default_value="true")
    pre_gripper_close_pause_ms_arg = DeclareLaunchArgument("pre_gripper_close_pause_ms", default_value="800")
    post_gripper_close_settle_ms_arg = DeclareLaunchArgument("post_gripper_close_settle_ms", default_value="0")
    lift_after_close_enable_arg = DeclareLaunchArgument("lift_after_close_enable", default_value="false")
    staged_lift_enable_arg = DeclareLaunchArgument("staged_lift_enable", default_value="false")
    staged_lift_first_step_arg = DeclareLaunchArgument("staged_lift_first_step", default_value="0.02")
    staged_lift_pause_ms_arg = DeclareLaunchArgument("staged_lift_pause_ms", default_value="0")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_move_group = LaunchConfiguration("start_move_group")

    ompl_yaml = {}
    try:
        pkg_share = get_package_share_directory("fairino3_v6_moveit2_config")
        ompl_path = os.path.join(pkg_share, "config", "ompl_planning.yaml")
        if os.path.exists(ompl_path):
            with open(ompl_path, "r") as f:
                ompl_yaml = yaml.safe_load(f) or {}
    except Exception:
        ompl_yaml = {}

    planning_pipelines_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        },
    }
    if isinstance(ompl_yaml, dict) and ompl_yaml:
        planning_pipelines_config["ompl"].update(ompl_yaml)

    trajectory_execution_config = {
        "trajectory_execution.allowed_execution_duration_scaling": 5.0,
        "trajectory_execution.allowed_goal_duration_margin": 15.0,
        "trajectory_execution.allowed_start_tolerance": 0.04,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            planning_pipelines_config,
            trajectory_execution_config,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(start_move_group),
    )

    pick_node = Node(
        package="fairino3_v6_moveit2_config",
        executable="pick_banana_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                "grasp_state_topic": LaunchConfiguration("grasp_state_topic"),
                "target_object_id": LaunchConfiguration("target_object_id"),
                "execute_once": LaunchConfiguration("execute_once"),
                "plan_only": LaunchConfiguration("plan_only"),
                "inspect_after_pregrasp": LaunchConfiguration("inspect_after_pregrasp"),
                "inspect_after_grasp": LaunchConfiguration("inspect_after_grasp"),
                "inspect_after_close": LaunchConfiguration("inspect_after_close"),
                "best_of_plans": LaunchConfiguration("best_of_plans"),
                "best_of_time_budget_sec": LaunchConfiguration("best_of_time_budget_sec"),
                "best_plan_metric": LaunchConfiguration("best_plan_metric"),
                "refine_at_pregrasp_enable": LaunchConfiguration("refine_at_pregrasp_enable"),
                "refine_at_pregrasp_cartesian": LaunchConfiguration("refine_at_pregrasp_cartesian"),
                "refine_at_pregrasp_min_xy": LaunchConfiguration("refine_at_pregrasp_min_xy"),
                "closed_loop_max_iterations": LaunchConfiguration("closed_loop_max_iterations"),
                "closed_loop_xyz_tolerance": LaunchConfiguration("closed_loop_xyz_tolerance"),
                "closed_loop_orientation_tolerance": LaunchConfiguration(
                    "closed_loop_orientation_tolerance"
                ),
                "closed_loop_max_translation_step": LaunchConfiguration(
                    "closed_loop_max_translation_step"
                ),
                "closed_loop_max_orientation_step": LaunchConfiguration(
                    "closed_loop_max_orientation_step"
                ),
                "add_table_collision": LaunchConfiguration("add_table_collision"),
                "avoid_collisions": LaunchConfiguration("avoid_collisions"),
                "descend_avoid_collisions": LaunchConfiguration("descend_avoid_collisions"),
                "reach_grasp_tolerance": LaunchConfiguration("reach_grasp_tolerance"),
                "target_x_offset": LaunchConfiguration("target_x_offset"),
                "target_y_offset": LaunchConfiguration("target_y_offset"),
                "target_z_offset": LaunchConfiguration("target_z_offset"),
                "target_z_override": LaunchConfiguration("target_z_override"),
                "pregrasp_z_offset": LaunchConfiguration("pregrasp_z_offset"),
                "grasp_z_offset": LaunchConfiguration("grasp_z_offset"),
                "finger_tip_z_offset": LaunchConfiguration("finger_tip_z_offset"),
                "lift_z_offset": LaunchConfiguration("lift_z_offset"),
                "lift_only": LaunchConfiguration("lift_only"),
                "lift_only_distance": LaunchConfiguration("lift_only_distance"),
                "eef_step": LaunchConfiguration("eef_step"),
                "min_fraction": LaunchConfiguration("min_fraction"),
                "min_grasp_height_above_target": LaunchConfiguration("min_grasp_height_above_target"),
                "auto_final_descend_enable": LaunchConfiguration("auto_final_descend_enable"),
                "auto_final_descend_margin": LaunchConfiguration("auto_final_descend_margin"),
                "auto_final_descend_max": LaunchConfiguration("auto_final_descend_max"),
                "force_extra_descend": LaunchConfiguration("force_extra_descend"),
                "min_eef_z": LaunchConfiguration("min_eef_z"),
                "descend_below_target": LaunchConfiguration("descend_below_target"),
                "single_descend_enable": LaunchConfiguration("single_descend_enable"),
                "single_descend_close_gripper": LaunchConfiguration("single_descend_close_gripper"),
                "single_descend_max_distance": LaunchConfiguration("single_descend_max_distance"),
                "single_descend_min_height_above_target": LaunchConfiguration(
                    "single_descend_min_height_above_target"
                ),
                "planning_time": LaunchConfiguration("planning_time"),
                "planning_attempts": LaunchConfiguration("planning_attempts"),
                "planning_pipeline_id": LaunchConfiguration("planning_pipeline_id"),
                "vel_scale": LaunchConfiguration("vel_scale"),
                "acc_scale": LaunchConfiguration("acc_scale"),
                "lift_vel_scale": LaunchConfiguration("lift_vel_scale"),
                "lift_acc_scale": LaunchConfiguration("lift_acc_scale"),
                "goal_pos_tolerance": LaunchConfiguration("goal_pos_tolerance"),
                "goal_ori_tolerance": LaunchConfiguration("goal_ori_tolerance"),
                "gripper_action_name": LaunchConfiguration("gripper_action_name"),
                "gripper_open_pos": LaunchConfiguration("gripper_open_pos"),
                "gripper_close_pos": LaunchConfiguration("gripper_close_pos"),
                "gripper_max_effort": LaunchConfiguration("gripper_max_effort"),
                "gripper_action_timeout_ms": LaunchConfiguration("gripper_action_timeout_ms"),
                "pregrasp_orientation_mode": LaunchConfiguration("pregrasp_orientation_mode"),
                "pregrasp_fixed_rpy": LaunchConfiguration("pregrasp_fixed_rpy"),
                "pregrasp_yaw_offset": LaunchConfiguration("pregrasp_yaw_offset"),
                "current_state_timeout": LaunchConfiguration("current_state_timeout"),
                "min_target_age_sec": LaunchConfiguration("min_target_age_sec"),
                "target_pos_epsilon": LaunchConfiguration("target_pos_epsilon"),
                "retry_interval_sec": LaunchConfiguration("retry_interval_sec"),
                "gripper_slow_close": LaunchConfiguration("gripper_slow_close"),
                "gripper_close_steps": LaunchConfiguration("gripper_close_steps"),
                "gripper_close_step_delay_ms": LaunchConfiguration("gripper_close_step_delay_ms"),
                "gripper_wait_result": LaunchConfiguration("gripper_wait_result"),
                "gripper_timeout_is_success": LaunchConfiguration("gripper_timeout_is_success"),
                "pre_gripper_close_pause_ms": LaunchConfiguration("pre_gripper_close_pause_ms"),
                "post_gripper_close_settle_ms": LaunchConfiguration("post_gripper_close_settle_ms"),
                "lift_after_close_enable": LaunchConfiguration("lift_after_close_enable"),
                "staged_lift_enable": LaunchConfiguration("staged_lift_enable"),
                "staged_lift_first_step": LaunchConfiguration("staged_lift_first_step"),
                "staged_lift_pause_ms": LaunchConfiguration("staged_lift_pause_ms"),
            },
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            start_move_group_arg,
            target_pose_topic_arg,
            grasp_state_topic_arg,
            target_object_id_arg,
            execute_once_arg,
            plan_only_arg,
            inspect_after_pregrasp_arg,
            inspect_after_grasp_arg,
            inspect_after_close_arg,
            best_of_plans_arg,
            best_of_time_budget_sec_arg,
            best_plan_metric_arg,
            refine_at_pregrasp_enable_arg,
            refine_at_pregrasp_cartesian_arg,
            refine_at_pregrasp_min_xy_arg,
            closed_loop_max_iterations_arg,
            closed_loop_xyz_tolerance_arg,
            closed_loop_orientation_tolerance_arg,
            closed_loop_max_translation_step_arg,
            closed_loop_max_orientation_step_arg,
            add_table_collision_arg,
            avoid_collisions_arg,
            descend_avoid_collisions_arg,
            reach_grasp_tolerance_arg,
            single_descend_enable_arg,
            single_descend_close_gripper_arg,
            single_descend_max_distance_arg,
            single_descend_min_height_above_target_arg,
            target_x_offset_arg,
            target_y_offset_arg,
            target_z_offset_arg,
            target_z_override_arg,
            pregrasp_z_offset_arg,
            grasp_z_offset_arg,
            finger_tip_z_offset_arg,
            lift_z_offset_arg,
            lift_only_arg,
            lift_only_distance_arg,
            eef_step_arg,
            min_fraction_arg,
            min_grasp_height_above_target_arg,
            auto_final_descend_enable_arg,
            auto_final_descend_margin_arg,
            auto_final_descend_max_arg,
            force_extra_descend_arg,
            min_eef_z_arg,
            descend_below_target_arg,
            planning_time_arg,
            planning_attempts_arg,
            planning_pipeline_id_arg,
            vel_scale_arg,
            acc_scale_arg,
            lift_vel_scale_arg,
            lift_acc_scale_arg,
            goal_pos_tolerance_arg,
            goal_ori_tolerance_arg,
            gripper_action_name_arg,
            gripper_open_pos_arg,
            gripper_close_pos_arg,
            gripper_max_effort_arg,
            gripper_action_timeout_ms_arg,
            pregrasp_orientation_mode_arg,
            pregrasp_fixed_rpy_arg,
            pregrasp_yaw_offset_arg,
            current_state_timeout_arg,
            min_target_age_sec_arg,
            target_pos_epsilon_arg,
            retry_interval_sec_arg,
            gripper_slow_close_arg,
            gripper_close_steps_arg,
            gripper_close_step_delay_ms_arg,
            gripper_wait_result_arg,
            gripper_timeout_is_success_arg,
            pre_gripper_close_pause_ms_arg,
            post_gripper_close_settle_ms_arg,
            lift_after_close_enable_arg,
            staged_lift_enable_arg,
            staged_lift_first_step_arg,
            staged_lift_pause_ms_arg,
            move_group_node,
            pick_node,
        ]
    )
