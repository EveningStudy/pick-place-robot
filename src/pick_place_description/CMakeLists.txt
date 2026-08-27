cmake_minimum_required(VERSION 3.16)
project(pick_place_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
