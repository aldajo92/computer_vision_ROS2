cmake_minimum_required(VERSION 3.8)
project(computer_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# C++ executable
add_executable(cv_node src/cv_node.cpp)
ament_target_dependencies(cv_node rclcpp sensor_msgs cv_bridge OpenCV)

# Install C++ executables
install(TARGETS
  cv_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  src/image_publisher_node.py
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/)

# Install dataset directory
install(DIRECTORY
  dataset/
  DESTINATION share/${PROJECT_NAME}/dataset/)

ament_package()
