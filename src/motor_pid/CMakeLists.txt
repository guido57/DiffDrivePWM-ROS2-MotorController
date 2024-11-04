cmake_minimum_required(VERSION 3.5)
project(motor_pid)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # Added nav_msgs for Odometry message

# Include directories
#include_directories(include)
include_directories(/opt/ros/humble/include)

# Find libpigpiod_if2.so
find_library(PIGPIO_LIBRARY pigpiod_if2 REQUIRED)
message(STATUS "Found pigpio: ${PIGPIO_LIBRARY}")

# Build the motor controller node
add_executable(motor_pid_node motor_pid_node.cpp)

# Link against the pigpiod_if2.so library
target_link_libraries(motor_pid_node ${PIGPIO_LIBRARY})

ament_target_dependencies(motor_pid_node rclcpp geometry_msgs nav_msgs)

install(TARGETS motor_pid_node
  DESTINATION lib/${PROJECT_NAME})

# Install any header files in include directory
#install(DIRECTORY include/
#  DESTINATION include/${PROJECT_NAME})

ament_package()
