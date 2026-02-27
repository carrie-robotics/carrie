cmake_minimum_required(VERSION 3.28.3)
project(carrie_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# find_package(<dependency> REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# On macOS, atomic ops are built into libc++ and there is no separate libatomic.
# The rosidl/FastRTPS build infrastructure unconditionally links -latomic, which
# fails on macOS. Create an empty stub library to satisfy the linker.
if(APPLE)
  file(WRITE "${CMAKE_BINARY_DIR}/atomic_stub.c" "int __atomic_stub_unused = 0;\n")
  add_library(atomic_shim STATIC "${CMAKE_BINARY_DIR}/atomic_stub.c")
  set_target_properties(atomic_shim PROPERTIES OUTPUT_NAME "atomic")
  link_directories("${CMAKE_BINARY_DIR}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectObjects.srv"
  DEPENDENCIES sensor_msgs vision_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
