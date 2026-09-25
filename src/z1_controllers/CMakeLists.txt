cmake_minimum_required(VERSION 3.8)
project(z1_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(z1_joint_trajectory_controller SHARED
  src/z1_joint_trajectory_controller.cpp)
target_include_directories(z1_joint_trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(z1_joint_trajectory_controller
  controller_interface
  control_msgs
  hardware_interface
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_action
  realtime_tools
  trajectory_msgs)

pluginlib_export_plugin_description_file(controller_interface z1_controllers_plugin.xml)

install(TARGETS z1_joint_trajectory_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(z1_joint_trajectory_controller)
ament_export_dependencies(controller_interface hardware_interface rclcpp rclcpp_action)

ament_package()
