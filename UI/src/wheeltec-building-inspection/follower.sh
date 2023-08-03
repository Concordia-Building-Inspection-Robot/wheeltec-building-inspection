#!/usr/bin/env bash
HOME="/home/$USER"
CATKIN_WS="$HOME/catkin_ws"
OPT_MELODIC_SETUP="/opt/ros/melodic/setup.bash" # ros system default workspace
ROS_PACKAGES_SETUP="$CATKIN_WS/devel/setup.bash"

# source primary catkin workspace, setup bash script and execute one launch script to rule them all
# PYTHONPATH append requited due to import issues
export PYTHONPATH=$PYTHONPATH:$CATKIN_WS/src/wheeltec-building-inspection && \
#!/bin/bash

# Set the ROS master URI and hostname
export ROS_MASTER_URI=http://192.168.1.101:11311
export ROS_HOSTNAME=concordia

# Extract the arguments
linear_x=$1
linear_y=$2
linear_z=$3
angular_x=$4
angular_y=$5
angular_z=$6

#run the executable file
./../../../../devel/lib/wheeltec-building-inspection-ui/follower $linear_x $linear_y $linear_z $angular_x $angular_y $angular_z

#python2.7 /home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/src/wheeltec-building-inspection/follower.py $linear_x $linear_y $linear_z $angular_x $angular_y $angular_z

# Publish the command velocity using rostopic
# rostopic pub -1 /cmd_vel geometry_msgs/Twist "{
#   linear: {
#     x: $linear_x,
#     y: $linear_y,
#     z: $linear_z
#   },
#   angular: {
#     x: $angular_x,
#     y: $angular_y,
#     z: $angular_z
#   }
# }"
