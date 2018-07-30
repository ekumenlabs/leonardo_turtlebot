cmake_minimum_required(VERSION 2.8.3)
project(auto_docking)

find_package(catkin REQUIRED COMPONENTS kobuki_msgs actionlib_msgs diagnostic_msgs)

catkin_package(CATKIN_DEPENDS kobuki_msgs actionlib_msgs diagnostic_msgs)

install(
  PROGRAMS
    scripts/auto_docking_client.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)