cmake_minimum_required(VERSION 3.28.3)
project(carrie_gz)

find_package(ament_cmake REQUIRED)

ament_python_install_package("carrie_gz")

install(
  DIRECTORY
    config
    config_gui
    launch
    rviz
    urdf
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
