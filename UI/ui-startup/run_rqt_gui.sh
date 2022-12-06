#!/usr/bin/env bash
HOME="/home/concordia"
CATKIN_WS="$HOME/catkin_ws"
OPT_MELODIC_SETUP="/opt/ros/melodic/setup.bash" # ros system default workspace
ROS_PACKAGES_SETUP="$CATKIN_WS/devel/setup.bash"

# source primary catkin workspace, setup bash script and execute one launch script to rule them all
export ROS_MASTER_URI=http://192.168.0.100:11311 && \
export export ROS_HOSTNAME=concordia && \
source $OPT_MELODIC_SETUP && source $ROS_PACKAGES_SETUP && rqt