cmake_minimum_required(VERSION 3.16)
project(pick_place_tasks)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(vision_pick_place_task src/vision_pick_place_task.cpp)
ament_target_dependencies(vision_pick_place_task
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  ros_gz_interfaces
  shape_msgs
  std_msgs
  std_srvs
)

install(TARGETS vision_pick_place_task DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
