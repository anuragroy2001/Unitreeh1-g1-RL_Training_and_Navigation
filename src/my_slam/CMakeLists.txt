cmake_minimum_required(VERSION 3.5)
project(my_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  behavior_tree
  launch
  maps
  rviz2
  config

  DESTINATION share/${PROJECT_NAME})

ament_package()
