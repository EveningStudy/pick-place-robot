cmake_minimum_required(VERSION 3.16)
project(pick_place_bringup)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
