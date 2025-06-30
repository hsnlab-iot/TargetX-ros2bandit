cmake_minimum_required(VERSION 3.8)
project(rmw_bandit)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rmw REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(rosidl_dynamic_typesupport REQUIRED)
find_package(rosidl_generator_c REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# Include dirs
include_directories(
  include
  ${ZMQ_INCLUDE_DIRS}
  /usr/include/nlohmann
)

# Library
add_library(rmw_bandit SHARED src/rmw_bandit.cpp)

# Link
target_link_libraries(rmw_bandit
  ${ZMQ_LIBRARIES}
  dl
)

# Dependencies for targets
ament_target_dependencies(rmw_bandit
  rmw
  rcutils
  rclcpp
  rosidl_typesupport_interface
  rosidl_dynamic_typesupport
  std_msgs
)

# Export the RMW implementation
ament_export_libraries(rmw_bandit)

ament_export_dependencies(
  rmw
  rmw_implementation
  rosidl_generator_c
  rcutils
  rosidl_typesupport_interface
  rosidl_dynamic_typesupport
  std_msgs
)

# set(resource_file "${CMAKE_CURRENT_BINARY_DIR}/rmw_implementation")
# file(WRITE "${resource_file}" "rmw_bandit\n")
# install(FILES "${resource_file}" DESTINATION share/ament_index/resource_index/rmw_implementation)

# Install library
install(TARGETS rmw_bandit
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Copy the resulting library to /host/build
add_custom_command(TARGET rmw_bandit POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:rmw_bandit> /host/build
)
# Register this RMW implementation with resource file
ament_index_register_resource("rmw_implementation" CONTENT "rmw_bandit")

ament_package()