cmake_minimum_required(VERSION 3.8)
project(joe_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
link_directories(/home/elf/mnt_ws/mavros_ws)
include_directories(/usr/include/libserial)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)  # 查找 mavros_msgs 包
find_package(geometry_msgs REQUIRED)  # 查找 geometry_msgs 包
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#target_include_directories(example_cpp PRIVATE /usr/include)

#include_directories(/home/elf/mnt_ws/mavros_ws)

add_executable(mid_360 src/mid_360.cpp)
ament_target_dependencies(mid_360 rclcpp nav_msgs geometry_msgs tf2
tf2_geometry_msgs)


add_executable(node_01 src/node_01.cpp)
ament_target_dependencies(node_01 rclcpp std_msgs)

add_executable(pwm_ctl src/pwm_ctl.cpp)
ament_target_dependencies(pwm_ctl rclcpp std_msgs)

add_executable(pwm_pub src/pwm_pub.cpp)
ament_target_dependencies(pwm_pub rclcpp std_msgs)

add_executable(interface_mavros src/interface_mavros.cpp)
ament_target_dependencies(interface_mavros rclcpp std_msgs mavros_msgs geometry_msgs)

add_executable(setpoint src/setpoint.cpp)
ament_target_dependencies(setpoint rclcpp geometry_msgs std_msgs mavros_msgs tf2
tf2_geometry_msgs)


add_executable(quad2Euler src/quad2Euler.cpp)
ament_target_dependencies(quad2Euler rclcpp geometry_msgs std_msgs mavros_msgs tf2
tf2_geometry_msgs)

target_link_libraries(node_01 serial)

install(TARGETS
  mid_360
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  setpoint
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  node_01
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  pwm_ctl 
  DESTINATION lib/${PROJECT_NAME}
  )

install(TARGETS
  pwm_pub 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  interface_mavros 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  quad2Euler 
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()