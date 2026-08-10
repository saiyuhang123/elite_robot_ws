cmake_minimum_required(VERSION 3.8)
project(gripper_control)

# Find ROS2 and required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# === Service definitions ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GripperCommand.srv"
)

# === Server node ===
add_executable(gripper_server src/gripper_server.cpp)
target_include_directories(gripper_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gripper_server rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gripper_server ${cpp_typesupport_target})

# === Install ===
install(TARGETS
  gripper_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
