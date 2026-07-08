cmake_minimum_required(VERSION 3.22)  # 指定最低 CMake 版本
project(interface)

# 找到 ROS 2 的必要依赖
find_package(rosidl_default_generators REQUIRED)

# 生成服务接口代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetPosition.srv"
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_package()