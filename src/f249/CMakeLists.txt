cmake_minimum_required(VERSION 3.8)
project(f249)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# Find dependencies
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Find libpigpiod_if2.so
find_library(PIGPIO_LIBRARY pigpiod_if2 REQUIRED)
message(STATUS "Found pigpio: ${PIGPIO_LIBRARY}")

# Build the f249 node
add_executable(f249_node src/f249driver.cpp)

# Link against the pigpiod_if2.so library
target_link_libraries(f249_node ${PIGPIO_LIBRARY})

ament_target_dependencies(f249_node rclcpp geometry_msgs)

install(TARGETS f249_node
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
