cmake_minimum_required(VERSION 3.22)
project(ros2help)

find_package(rclcpp REQUIRED)
add_executable(ros2help ros2help.cpp)
target_link_libraries(ros2help rclcpp::rclcpp)
install(TARGETS
  ros2help
  DESTINATION lib/${PROJECT_NAME}
)