cmake_minimum_required(VERSION 3.22)
project(fairino_description)

find_package(ament_cmake REQUIRED)

install(
  FILES
    base_link.STL
    forearm_link.STL
    shoulder_link.STL
    upperarm_link.STL
    wrist1_link.STL
    wrist2_link.STL
    wrist3_link.STL
  DESTINATION share/${PROJECT_NAME}/meshes/fairino3_v6
)

install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../fairino3_v6.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
