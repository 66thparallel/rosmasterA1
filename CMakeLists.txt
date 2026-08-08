cmake_minimum_required(VERSION 3.8)
project(rosmaster_a1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(path_planner_node src/path_planner_node.cpp)
ament_target_dependencies(path_planner_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
)

add_executable(pure_pursuit_controller_node src/pure_pursuit_controller_node.cpp)
target_include_directories(pure_pursuit_controller_node PRIVATE include)
ament_target_dependencies(pure_pursuit_controller_node
  ackermann_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

add_executable(vehicle_simulator_node src/vehicle_simulator_node.cpp)
target_include_directories(vehicle_simulator_node PRIVATE include)
ament_target_dependencies(vehicle_simulator_node
  ackermann_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(metrics_logger_node src/metrics_logger_node.cpp)
ament_target_dependencies(metrics_logger_node
  diagnostic_msgs
  rclcpp
)

install(TARGETS
  path_planner_node
  pure_pursuit_controller_node
  vehicle_simulator_node
  metrics_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pure_pursuit test/test_pure_pursuit.cpp)
  target_include_directories(test_pure_pursuit PRIVATE include)

  ament_add_gtest(test_vehicle_kinematics test/test_vehicle_kinematics.cpp)
  target_include_directories(test_vehicle_kinematics PRIVATE include)
endif()

ament_package()