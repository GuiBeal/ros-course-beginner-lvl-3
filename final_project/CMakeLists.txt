cmake_minimum_required(VERSION 3.8)
project(final_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_library(turtle_controller SHARED src/turtle_controller.cpp)
ament_target_dependencies(turtle_controller
  rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle turtlesim geometry_msgs my_robot_interfaces
)
rclcpp_components_register_nodes(turtle_controller "final_project::TurtleController")

add_library(turtlebot_controller SHARED src/turtlebot_controller.cpp)
ament_target_dependencies(turtlebot_controller
  rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle geometry_msgs my_robot_interfaces
)
rclcpp_components_register_nodes(turtlebot_controller "final_project::TurtlebotController")

install(TARGETS
  turtle_controller
  turtlebot_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
