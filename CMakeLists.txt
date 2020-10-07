cmake_minimum_required(VERSION 3.5)
project(camera1394)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

set(dependencies
  "rclcpp")

set(srv_files
  "srv/GetCameraRegisters.srv"
  "srv/SetCameraRegisters.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)
ament_export_dependencies(rosidl_default_runtime)


# add executable
set(NODE_NAME "camera1394_node")
file(GLOB MY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
add_executable(${NODE_NAME} ${MY_SOURCES})
ament_target_dependencies(${NODE_NAME} ${dependencies})

install(TARGETS
  camera1394_node 
  DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(camera1394_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
