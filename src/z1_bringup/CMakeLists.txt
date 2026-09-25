cmake_minimum_required(VERSION 3.8)
project(z1_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Shared launch helpers (z1_bringup.gz_sim), used by the two Gazebo bringup files.
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
