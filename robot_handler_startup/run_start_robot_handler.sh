#!/usr/bin/env bash
HOME="/home/wheeltec"
CATKIN_WS="$HOME/catkin_workspace"
OPT_MELODIC_SETUP="/opt/ros/melodic/setup.bash" # ros system default workspace
ROS_PACKAGES_SETUP="$CATKIN_WS/devel/setup.bash"
ROSLAUNCH_FILE="$CATKIN_WS/src/wheeltec-building-inspection/robot_handler_startup/start_robot_handler.launch"

# source primary catkin workspace, setup bash script and execute one launch script to rule them all
export PYTHONPATH=$PYTHONPATH:$HOME/catkin_workspace/src/wheeltec-building-inspection && \
source $OPT_MELODIC_SETUP && source $ROS_PACKAGES_SETUP && roslaunch $ROSLAUNCH_FILE
