cmake_minimum_required(VERSION 3.22)
project(fairino3_v6_moveit2_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(pick_banana_node src/pick_banana_node.cpp)
ament_target_dependencies(
  pick_banana_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_msgs
  control_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/yolov8_overlay_node.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../isaac_fr3_sim/scripts/trajectory_action_proxy.py
  ${CMAKE_CURRENT_SOURCE_DIR}/../../isaac_fr3_sim/scripts/gripper_action_proxy.py
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS pick_banana_node DESTINATION lib/${PROJECT_NAME})
