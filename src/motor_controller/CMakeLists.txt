cmake_minimum_required(VERSION 3.5)
project(motor_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
#include_directories(include)

# Find libpigpiod_if2.so
find_library(PIGPIO_LIBRARY pigpiod_if2 REQUIRED)
message(STATUS "Found pigpio: ${PIGPIO_LIBRARY}")

# Build the motor controller node
add_executable(motor_controller_node motor_controller_node.cpp)

# Link against the pigpiod_if2.so library
target_link_libraries(motor_controller_node ${PIGPIO_LIBRARY})

ament_target_dependencies(motor_controller_node rclcpp geometry_msgs)

install(TARGETS motor_controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
