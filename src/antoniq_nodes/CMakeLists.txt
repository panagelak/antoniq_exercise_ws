cmake_minimum_required(VERSION 3.5)
project(antoniq_nodes CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

# find dependencies
set(dependencies
    ament_cmake
    ament_cmake_python
    rclcpp
    rclcpp_action
    std_srvs
    std_msgs
    nav_msgs
    tf2_geometry_msgs
    tf2_ros
    tf2_eigen
    robot_localization
    visualization_msgs
    nav2_util
    nav2_msgs
    geometry_msgs
    sensor_msgs
    control_toolbox
    antoniq_interfaces
    ament_index_cpp)
foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

# c++ executables
add_executable(docking_server_node nodes/docking_server_node.cpp)
ament_target_dependencies(docking_server_node ${dependencies})
add_executable(rotate_heading_server_node nodes/rotate_heading_server_node.cpp)
ament_target_dependencies(rotate_heading_server_node ${dependencies})
add_executable(antoniq_mission_node nodes/antoniq_mission_node.cpp)
ament_target_dependencies(antoniq_mission_node ${dependencies})
add_executable(obstacle_monitor_node nodes/obstacle_monitor_node.cpp)
ament_target_dependencies(obstacle_monitor_node ${dependencies})
# INSTALL
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

# install c++ executables
install(TARGETS docking_server_node rotate_heading_server_node antoniq_mission_node obstacle_monitor_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION
  lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # clang-format (config at repo root .clang-format, checked in CI) is the authoritative C++
  # formatter here; ament_uncrustify enforces a different, conflicting style convention, so it's
  # disabled to avoid two formatting tools fighting over the same files.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission_logic test/test_mission_logic.cpp)
  target_include_directories(test_mission_logic PRIVATE include)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_mission_node_launch.py
    TARGET test_mission_node_launch
    TIMEOUT 30)
endif()

# EXPORTS
ament_export_include_directories(include)

ament_export_dependencies(${dependencies})

ament_package()
