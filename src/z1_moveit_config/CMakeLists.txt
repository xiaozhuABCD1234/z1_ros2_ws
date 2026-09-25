cmake_minimum_required(VERSION 3.8)
project(z1_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Shared configuration module (z1_moveit_config.moveit_config), so every launch
# file builds an identical MoveIt parameter set.
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
