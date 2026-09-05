cmake_minimum_required(VERSION 3.22)
project(fr3_world)

find_package(ament_cmake REQUIRED)

install(FILES custom_room.world DESTINATION share/${PROJECT_NAME})
install(DIRECTORY room DESTINATION share/${PROJECT_NAME})

ament_package()
