cmake_minimum_required(VERSION 2.8.3)
project(wireless_watcher)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/watcher_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
