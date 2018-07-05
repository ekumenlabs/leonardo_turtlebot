#!/bin/bash

source /opt/ros/kinetic/setup.bash

export TURTLEBOT_3D_SENSOR=kinect
export TURTLEBOT_MAP_FILE=/home/turtlebot/map/attic.yaml

roslaunch leonardo_bringup leonardo.launch
