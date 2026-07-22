cmake_minimum_required(VERSION 3.8)
project(antoniq_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  params
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
