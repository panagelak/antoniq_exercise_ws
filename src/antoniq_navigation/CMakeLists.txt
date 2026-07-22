cmake_minimum_required(VERSION 3.5)
project(antoniq_navigation)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config behavior_trees launch maps DESTINATION share/${PROJECT_NAME})
