from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fairino3_v6_robot", package_name="fairino3_v6_moveit2_config").to_moveit_configs()
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_sim_time = LaunchConfiguration("use_sim_time")

    generated = generate_moveit_rviz_launch(moveit_config)
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(SetParameter(name="use_sim_time", value=use_sim_time))
    for entity in generated.entities:
        ld.add_action(entity)
    return ld
