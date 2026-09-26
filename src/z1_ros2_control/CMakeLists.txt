cmake_minimum_required(VERSION 3.8)
project(z1_ros2_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(z1_ros2_control SHARED
  src/z1_system.cpp
  src/z1_udp_client.cpp)
target_include_directories(z1_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# The vendored Unitree header lives outside include/ so that it stays visibly
# third-party. See thirdparty/unitree/z1/PROVENANCE.md.
target_include_directories(z1_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty>
  $<INSTALL_INTERFACE:thirdparty>)
ament_target_dependencies(z1_ros2_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface z1_ros2_control_plugin.xml)

install(TARGETS z1_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY thirdparty/ DESTINATION thirdparty)
install(PROGRAMS scripts/z1_ctrl_mock.py
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include thirdparty)
ament_export_libraries(z1_ros2_control)
ament_export_dependencies(hardware_interface rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_z1_protocol test/test_z1_protocol.cpp)
  target_link_libraries(test_z1_protocol z1_ros2_control)
  ament_target_dependencies(test_z1_protocol hardware_interface rclcpp)
endif()

ament_package()
