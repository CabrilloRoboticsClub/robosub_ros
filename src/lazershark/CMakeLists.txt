cmake_minimum_required(VERSION 3.5)
project(lazershark)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

# C++ executable
add_executable(cpp_test auv/pi/cpp_test.cpp)
ament_target_dependencies(cpp_test rclcpp std_msgs)

# Install C++ node
install(TARGETS cpp_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()
