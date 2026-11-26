cmake_minimum_required(VERSION 3.8)
project(bno085_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Protocol parsing and serial I/O, kept free of ROS so they can be unit tested.
add_library(bno085_core STATIC
  src/protocol.cpp
  src/serial_port.cpp
)
target_include_directories(bno085_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(bno085_node src/bno085_node.cpp)
target_link_libraries(bno085_node bno085_core)
ament_target_dependencies(bno085_node rclcpp geometry_msgs sensor_msgs)

install(TARGETS bno085_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol bno085_core)
endif()

ament_package()
