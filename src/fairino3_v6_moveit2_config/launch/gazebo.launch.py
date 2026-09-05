from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
import os
from pathlib import Path


def generate_launch_description():
    moveit_share = Path(get_package_share_directory("fairino3_v6_moveit2_config"))
    world_share = Path(get_package_share_directory("fr3_world"))
    fairino_description_share = Path(get_package_share_directory("fairino_description"))

    urdf_xacro_path = moveit_share / "config" / "fairino3_v6_robot.urdf.xacro"
    initial_positions_path = moveit_share / "config" / "initial_positions.yaml"
    controllers_path = moveit_share / "config" / "ros2_controllers.yaml"

    default_world = world_share / "custom_room.world"

    table_center_x = -1.87817
    table_center_y = -2.66631
    table_surface_size = 0.913
    table_top_z = 0.755 + 0.02

    base_radius = 0.064
    table_edge_clearance = 0.02
    corner_offset = table_surface_size / 2.0 - (base_radius + table_edge_clearance)

    default_robot_x = table_center_x - corner_offset
    default_robot_y = table_center_y - corner_offset
    default_robot_z = table_top_z
    default_robot_yaw = 3.14159

    robot_description = Command(
        [
            "xacro ",
            str(urdf_xacro_path),
            " initial_positions_file:=",
            str(initial_positions_path),
            " controllers_file:=",
            str(controllers_path),
            " use_gazebo_ros2_control:=true",
            " use_fixed_base:=true",
            " add_robotiq_gripper:=",
            LaunchConfiguration("add_robotiq_gripper"),
            " enable_grasp_fix:=",
            LaunchConfiguration("enable_grasp_fix"),
        ]
    )

    gui_arg = DeclareLaunchArgument("gui", default_value="true")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    world_arg = DeclareLaunchArgument("world", default_value=str(default_world))
    robot_x_arg = DeclareLaunchArgument("robot_x", default_value=str(default_robot_x))
    robot_y_arg = DeclareLaunchArgument("robot_y", default_value=str(default_robot_y))
    robot_z_arg = DeclareLaunchArgument("robot_z", default_value=str(default_robot_z))
    robot_yaw_arg = DeclareLaunchArgument("robot_yaw", default_value=str(default_robot_yaw))
    yolo_enable_arg = DeclareLaunchArgument("yolo_enable", default_value="true")
    yolo_weights_arg = DeclareLaunchArgument("yolo_weights", default_value="yolov8n.pt")
    yolo_image_in_arg = DeclareLaunchArgument("yolo_image_in", default_value="/wrist_camera/image_raw")
    yolo_image_out_arg = DeclareLaunchArgument("yolo_image_out", default_value="/yolo/dbg_image")
    yolo_conf_arg = DeclareLaunchArgument("yolo_conf", default_value="0.25")
    yolo_iou_arg = DeclareLaunchArgument("yolo_iou", default_value="0.7")
    yolo_device_arg = DeclareLaunchArgument("yolo_device", default_value="cpu")
    yolo_classes_arg = DeclareLaunchArgument("yolo_classes", default_value="[]")
    yolo_target_frame_arg = DeclareLaunchArgument("yolo_target_frame", default_value="base_link")
    add_robotiq_gripper_arg = DeclareLaunchArgument(
        "add_robotiq_gripper", default_value="true"
    )
    enable_grasp_fix_arg = DeclareLaunchArgument("enable_grasp_fix", default_value="true")

    use_sim_time = LaunchConfiguration("use_sim_time")
    add_robotiq_gripper = LaunchConfiguration("add_robotiq_gripper")
    enable_grasp_fix = LaunchConfiguration("enable_grasp_fix")

    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.pathsep.join(
            [
                str(world_share),
                str(fairino_description_share.parent),
                str(Path.home() / ".gazebo" / "models"),
                "/usr/share/gazebo-11/models",
            ]
        ),
    )

    set_gazebo_model_database_uri = SetEnvironmentVariable(
        name="GAZEBO_MODEL_DATABASE_URI",
        value="",
    )

    set_gazebo_log_path = SetEnvironmentVariable(name="GAZEBO_LOG_PATH", value="/tmp/gazebo")
    set_ros_log_dir = SetEnvironmentVariable(name="ROS_LOG_DIR", value="/tmp/ros_log")

    robotiq_plugin_lib = os.path.join(get_package_prefix("robotiq_description"), "lib")
    gazebo_plugin_path_parts = [
        p for p in os.environ.get("GAZEBO_PLUGIN_PATH", "").split(os.pathsep) if p
    ]
    if robotiq_plugin_lib not in gazebo_plugin_path_parts:
        gazebo_plugin_path_parts.append(robotiq_plugin_lib)
    gazebo_plugin_path = os.pathsep.join(gazebo_plugin_path_parts)

    set_gazebo_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=gazebo_plugin_path,
    )

    gzserver_env = {
        "HOME": "/tmp",
        "GAZEBO_PLUGIN_PATH": gazebo_plugin_path,
    }

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            LaunchConfiguration("world"),
        ],
        additional_env=gzserver_env,
        output="screen",
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        condition=IfCondition(LaunchConfiguration("gui")),
        additional_env={"HOME": "/tmp", "GAZEBO_PLUGIN_PATH": gazebo_plugin_path},
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "fairino3_v6_robot",
            "-x",
            LaunchConfiguration("robot_x"),
            "-y",
            LaunchConfiguration("robot_y"),
            "-z",
            LaunchConfiguration("robot_z"),
            "-Y",
            LaunchConfiguration("robot_yaw"),
            "-timeout",
            "120.0",
        ],
        output="screen",
    )

    world_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            LaunchConfiguration("robot_x"),
            "--y",
            LaunchConfiguration("robot_y"),
            "--z",
            LaunchConfiguration("robot_z"),
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            LaunchConfiguration("robot_yaw"),
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    fairino3_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fairino3_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(add_robotiq_gripper),
    )

    initial_upright_pose = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "export ROS_LOG_DIR=/tmp/ros_log && "
            "ros2 topic pub --once /fairino3_controller/joint_trajectory "
            "trajectory_msgs/msg/JointTrajectory "
            "\"{joint_names: [j1, j2, j3, j4, j5, j6], points: [{positions: [1.2, -1.2, 1.0, -1.8, -1.57, 0.0], time_from_start: {sec: 2}}]}\"",
        ],
        output="screen",
    )

    delayed_spawn_and_controllers = TimerAction(
        period=2.0,
        actions=[
            world_to_base_link_tf,
            spawn_entity,
            joint_state_broadcaster_spawner,
            fairino3_controller_spawner,
            gripper_controller_spawner,
        ],
    )

    delayed_initial_pose = TimerAction(period=5.0, actions=[initial_upright_pose])

    yolov8_script = PathJoinSubstitution(
        [
            FindPackagePrefix("fairino3_v6_moveit2_config"),
            "lib",
            "fairino3_v6_moveit2_config",
            "yolov8_overlay_node.py",
        ]
    )
    yolov8_overlay = ExecuteProcess(
        cmd=[
            "python3",
            yolov8_script,
            "--ros-args",
            "-p", ["use_sim_time:=", use_sim_time],
            "-p", ["image_in:=", LaunchConfiguration("yolo_image_in")],
            "-p", ["image_out:=", LaunchConfiguration("yolo_image_out")],
            "-p", ["weights:=", LaunchConfiguration("yolo_weights")],
            "-p", ["conf:=", LaunchConfiguration("yolo_conf")],
            "-p", ["iou:=", LaunchConfiguration("yolo_iou")],
            "-p", ["device:=", LaunchConfiguration("yolo_device")],
            "-p", ["classes:=", LaunchConfiguration("yolo_classes")],
            "-p", ["target_frame:=", LaunchConfiguration("yolo_target_frame")],
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("yolo_enable")),
    )

    delayed_yolo = TimerAction(period=7.0, actions=[yolov8_overlay])

    return LaunchDescription(
        [
            gui_arg,
            use_sim_time_arg,
            world_arg,
            robot_x_arg,
            robot_y_arg,
            robot_z_arg,
            robot_yaw_arg,
            yolo_enable_arg,
            yolo_weights_arg,
            yolo_image_in_arg,
            yolo_image_out_arg,
            yolo_conf_arg,
            yolo_iou_arg,
            yolo_device_arg,
            yolo_classes_arg,
            yolo_target_frame_arg,
            add_robotiq_gripper_arg,
            enable_grasp_fix_arg,
            set_gazebo_model_path,
            set_gazebo_model_database_uri,
            set_gazebo_log_path,
            set_ros_log_dir,
            set_gazebo_plugin_path,
            gzserver,
            gzclient,
            robot_state_publisher,
            delayed_spawn_and_controllers,
            delayed_initial_pose,
            delayed_yolo,
        ]
    )
