#!/usr/bin/env python
# This script can be used to monitor the acceleration of the rover for collision detection
# however it is depricated, and has been replaced by cpp_monitor

import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import String

def callback(data):
    acc = math.sqrt(data.linear_acceleration.x ** 2 + data.linear_acceleration.y ** 2)

    global prev_acc 
    global jerk_max 
    global pub

    jerk = acc - prev_acc

    if jerk > jerk_max: 
        print("collision detected")
        pub.publish(String("halt 1"))

    prev_acc = acc

def main():
    global prev_acc
    global jerk_max 
    global pub

    jerk_max = 4.0
    prev_acc = 0

    rospy.init_node("acceleration_monitor", anonymous=False)
    sub = rospy.Subscriber("imu", Imu, callback)
    pub = rospy.Publisher("robot_handler_cmd", String, queue_size=10)
    
    rospy.spin()

main()
