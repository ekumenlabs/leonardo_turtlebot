#!/bin/bash

LEONARDO_TURTLEBOT_ROOT=/home/turtlebot/leonardo_turtlebot/

source $LEONARDO_TURTLEBOT_ROOT/devel/setup.bash

export TURTLEBOT_3D_SENSOR=kinect
export TURTLEBOT_MAP_FILE=$LEONARDO_TURTLEBOT_ROOT/src/leonardo_bringup/map/attic.yaml

roslaunch leonardo_bringup leonardo.launch
