cmake_minimum_required(VERSION 3.5)
project(wireless_watcher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
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
  diagnostic_updater
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

# Specify install targets
ament_package()
