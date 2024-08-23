cmake_minimum_required(VERSION 3.8)
project(controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(include)

# Add library for the StanleyControllerNode
add_library(stanley_controller_lib src/stanley.cpp)
ament_target_dependencies(stanley_controller_lib rclcpp geometry_msgs)

# Add executable
add_executable(stanley_controller src/stanley_main.cpp)
target_link_libraries(stanley_controller stanley_controller_lib)
ament_target_dependencies(stanley_controller rclcpp geometry_msgs)

# Install targets
install(TARGETS
  stanley_controller
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Linting (ignore for now)
  # find_package(ament_lint_auto REQUIRED)

  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  # Add gtest
  ament_add_gtest(test_stanley_controller test/test_stanley_controller.cpp)
  target_link_libraries(test_stanley_controller stanley_controller_lib)
  ament_target_dependencies(test_stanley_controller rclcpp geometry_msgs)
endif()

ament_package()
