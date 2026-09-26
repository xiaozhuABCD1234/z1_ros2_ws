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
find_package(rosidl_default_generators REQUIRED)

# The FSM services (`~/set_fsm_state`, `~/get_fsm_state`) are served from the
# hardware component itself, because z1_ctrl only ever answers the one client
# that binds port 8072 - a separate node could not reach it.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetFSMState.srv"
  "srv/SetFSMState.srv")

# The CMake target cannot be called ${PROJECT_NAME}: rosidl_generate_interfaces
# already creates a custom target with that name. OUTPUT_NAME keeps the shared
# object itself at libz1_ros2_control.so, which is what the plugin description
# (z1_ros2_control_plugin.xml) and every <plugin> in the URDF refer to.
add_library(${PROJECT_NAME}_plugin SHARED
  src/z1_system.cpp
  src/z1_udp_client.cpp)
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# The vendored Unitree header lives outside include/ so that it stays visibly
# third-party. See thirdparty/unitree/z1/PROVENANCE.md.
target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty>
  $<INSTALL_INTERFACE:thirdparty>)
ament_target_dependencies(${PROJECT_NAME}_plugin
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle)
# The component serves the FSM services defined above, so it needs their C++
# type support.
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME}__rosidl_typesupport_cpp)

pluginlib_export_plugin_description_file(hardware_interface z1_ros2_control_plugin.xml)

install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY thirdparty/ DESTINATION thirdparty)
install(PROGRAMS scripts/z1_ctrl_mock.py
  DESTINATION lib/${PROJECT_NAME})

# No ament_export_libraries(): pluginlib loads the plugin through the description
# file above, so nothing links against it. (Exporting it would also have to name
# the *_plugin target, which would not match the installed .so.)
ament_export_include_directories(include thirdparty)
ament_export_dependencies(hardware_interface rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_z1_protocol test/test_z1_protocol.cpp)
  target_link_libraries(test_z1_protocol ${PROJECT_NAME}_plugin)
  ament_target_dependencies(test_z1_protocol hardware_interface rclcpp)
endif()

ament_package()
