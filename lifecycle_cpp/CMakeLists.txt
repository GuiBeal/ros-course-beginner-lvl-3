cmake_minimum_required(VERSION 3.8)
project(lifecycle_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp rclcpp_lifecycle example_interfaces)

add_executable(move_robot_server src/move_robot_server.cpp)
ament_target_dependencies(move_robot_server rclcpp rclcpp_action rclcpp_lifecycle my_robot_interfaces)

add_executable(lifecycle_node_manager src/lifecycle_node_manager.cpp)
ament_target_dependencies(lifecycle_node_manager rclcpp lifecycle_msgs)

add_executable(move_robot_startup src/move_robot_startup.cpp)
ament_target_dependencies(move_robot_startup rclcpp lifecycle_msgs)

install(TARGETS
  number_publisher
  move_robot_server
  lifecycle_node_manager
  move_robot_startup
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
