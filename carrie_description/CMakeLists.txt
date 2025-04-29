cmake_minimum_required(VERSION 3.28.3)
project(carrie_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY
    launch
    meshes
    urdf
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
endif()

ament_package()
