cmake_minimum_required(VERSION 3.8)
project(elite_polish_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen_kdl REQUIRED)
find_package(eli_common_interface REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# =============================================
# Bridge node: Topic -> Action adapter
# =============================================
add_executable(elite_joint_trajectory_bridge
  src/elite_joint_trajectory_bridge.cpp
)

ament_target_dependencies(elite_joint_trajectory_bridge
  rclcpp rclcpp_action trajectory_msgs control_msgs)

install(TARGETS
  elite_joint_trajectory_bridge
  DESTINATION lib/${PROJECT_NAME})

# =============================================
# TODO: More executables to be added:
#   - eliteCamera3DSolver     (vision solver)
# =============================================

# =============================================
# Main state machine: ysURForceAppControl
# =============================================
add_executable(ysURForceAppControl
  src/ysURForceAppNode.cpp
  src/ysURForceAppControl.cpp
)

ament_target_dependencies(ysURForceAppControl
  rclcpp sensor_msgs geometry_msgs Eigen3 orocos_kdl
  trac_ik_lib std_srvs PCL
  trajectory_msgs std_msgs eli_common_interface)

install(TARGETS
  ysURForceAppControl
  DESTINATION lib/${PROJECT_NAME})

# =============================================
# Vision solver: 3D camera point cloud processing
# =============================================
add_executable(ysCamera3DSolver
  src/ysCamera3DSolverNode.cpp
  src/ysCamera3DSolver.cpp
)

ament_target_dependencies(ysCamera3DSolver
  rclcpp sensor_msgs geometry_msgs Eigen3 orocos_kdl
  trac_ik_lib std_srvs PCL pcl_conversions tf2_eigen_kdl)

install(TARGETS
  ysCamera3DSolver
  DESTINATION lib/${PROJECT_NAME})

# =============================================
# App command publisher
# =============================================
add_executable(ysAppCommand
  src/ysAppCommandNode.cpp
)

ament_target_dependencies(ysAppCommand
  rclcpp Eigen3 std_msgs)

install(TARGETS
  ysAppCommand
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# Install config files (polish_params.yaml)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

# Install template PCD files
install(DIRECTORY
  share
  DESTINATION share/${PROJECT_NAME})
