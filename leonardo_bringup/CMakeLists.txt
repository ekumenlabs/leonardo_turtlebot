cmake_minimum_required(VERSION 2.8.3)
project(leonardo_bringup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(
    PROGRAMS
      scripts/turtlebot_addr.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY icons
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY interactions
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
