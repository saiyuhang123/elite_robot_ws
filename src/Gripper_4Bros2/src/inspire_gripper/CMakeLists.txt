cmake_minimum_required(VERSION 3.8)
project(inspire_gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(service_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(serial REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)  # 确保添加 std_msgs
find_package(rosidl_default_generators REQUIRED)#改
include_directories(include ${ament_INCLUDE_DIRS})

# 添加可执行文件
add_executable(Gripper_control_node src/Gripper_control.cpp)

# 添加依赖
ament_target_dependencies(Gripper_control_node
  rclcpp
  service_interfaces
  serial
  std_msgs
)

# 安装目标
install(TARGETS
  Gripper_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# 测试和代码检查
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 最后，调用 ament_package()
ament_package()

