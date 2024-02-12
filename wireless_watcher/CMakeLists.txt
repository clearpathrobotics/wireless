cmake_minimum_required(VERSION 3.5)
project(wireless_watcher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wireless_msgs REQUIRED)

# Add executable
add_executable(wireless_watcher src/wireless_watcher.cpp)

target_include_directories(
  wireless_watcher
  PRIVATE
  include
)

# Link against ROS 2 libraries
ament_target_dependencies(wireless_watcher
  rclcpp
  std_msgs
  wireless_msgs
)

# Install executable
install(TARGETS
  wireless_watcher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclcpp std_msgs wireless_msgs)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Specify install targets
ament_package()
