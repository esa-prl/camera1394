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

include_directories(include)

set(dependencies
  "rclcpp")

# add executable
# add_executable(camera1394_node src/camera1394_node.cpp)
# ament_target_dependencies(camera1394_node ${dependencies})

# add executable
set(NODE_NAME "camera1394_node")
file(GLOB MY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
add_executable(${NODE_NAME} ${MY_SOURCES})
ament_target_dependencies(${NODE_NAME} ${dependencies})

install(TARGETS
  camera1394_node 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
