# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import SetParameter


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("fairino3_v6_robot", package_name="fairino3_v6_moveit2_config").to_moveit_configs()
#     use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
#     use_sim_time = LaunchConfiguration("use_sim_time")

#     generated = generate_move_group_launch(moveit_config)
#     ld = LaunchDescription()
#     ld.add_action(use_sim_time_arg)
#     ld.add_action(SetParameter(name="use_sim_time", value=use_sim_time))
#     for entity in generated.entities:
#         ld.add_action(entity)
#     return ld


from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "fairino3_v6_robot", package_name="fairino3_v6_moveit2_config"
    ).to_moveit_configs()
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    # Isaac may advance simulation time slower than wall time on this GPU.
    trajectory_execution_config = {
        "trajectory_execution.allowed_execution_duration_scaling": 100.0,
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
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(move_group_node)
    return ld
