#!/usr/bin/env python

import rospy

from std_msgs.msg import String

import multiprocessing
import os

class RobotState():
    IDLE = 1
    MAPPING = 2
    NAVIGATION = 3
    SLAM = 4 # currently unused

class RobotHandler():
    def __init__(self):
        self.currentState = RobotState.IDLE
        self.rosLaunchThread = None

    def run_launch_file(self, package, launch_file):
        success = True
        try:
            print('start ' + launch_file)

            self.rosLaunchThread = multiprocessing.Process(
                target=lambda: os.system('roslaunch ' + package + ' ' + launch_file)
            )
            self.rosLaunchThread.start()
        except():
            print('exception occurred running ' + launch_file)
            success = False

        return success

    def start_mapping(self):
        if (self.currentState == RobotState.IDLE):
            success = self.run_launch_file('turn_on_wheeltec_robot', 'mapping.launch')
            if success:
                self.currentState = RobotState.MAPPING
        else:
            # TODO: Display pop-up telling user that robot must be idle to start mapping
            print('')

    def save_map(self):
        if (self.currentState == RobotState.Mapping):
            success = self.run_launch_file('turn_on_wheeltec_robot', 'map_saver.launch')
            if success:
                # TODO: Display pop-up telling user that map was saved
                print('successfully saved map')
        else:
            # TODO: Display pop-up telling user that map cannot be saved without mapping running
            print('')

    def start_navigation(self):
        if (self.currentState == RobotState.IDLE):
            success = self.run_launch_file('turn_on_wheeltec_robot', 'navigation.launch')
            if success:
                self.currentState = RobotState.NAVIGATION
        else:
            # TODO: Display pop-up telling user that robot must be idle to start navigation
            print('')

    def stop_all(self):
        if self.currentState != RobotState.IDLE and self.rosLaunchThread is not None:
            self.rosLaunchThread.stop()
            self.currentState = RobotState.IDLE

if __name__ == '__main__':
    def main():
        rospy.spin()

    def command_handler(cmd):
        if cmd == 'start_mapping':
            robot_handler.start_mapping()
        elif cmd == 'save_map':
            robot_handler.save_map()
        elif cmd == 'start_nav':
            robot_handler.start_navigation()
        elif cmd == 'stop_all':
            robot_handler.stop_all()

    robot_handler = RobotHandler()

    node_name = 'robot_handler'
    rospy.init_node(node_name, anonymous=False)  # only allow one node of this type
    rospy.loginfo('Initialized "' + node_name + '" node for pub/sub/service functionality')

    command_topic = '/robot_handler_cmd'
    rospy.loginfo('Beginning to subscribe to "' + command_topic + '" topic')
    sub = rospy.Subscriber(command_topic, String, command_handler)

    main()
