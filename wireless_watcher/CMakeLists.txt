cmake_minimum_required(VERSION 2.8.3)
project(wireless_watcher)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  nodes/watcher_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
