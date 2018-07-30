cmake_minimum_required(VERSION 2.8.3)
project(leonardo_battery_guard)

find_package(catkin REQUIRED COMPONENTS kobuki_msgs actionlib_msgs diagnostic_msgs)

catkin_package(CATKIN_DEPENDS kobuki_msgs actionlib_msgs diagnostic_msgs)

catkin_install_python(
  PROGRAMS
    scripts/battery_guard.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)