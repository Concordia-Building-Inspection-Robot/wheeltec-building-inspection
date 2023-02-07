#!/usr/bin/env python

import os

import rospy

from std_msgs.msg import String
from actionlib_msgs.msg import GoalID

from utils.Definitions import ROBOT_HOME_DIRECTORY, ROBOT_CAP_SAVE_DIRECTORY
from utils.SubProcessManager import *

class RobotState():
    IDLE = 0
    NORMAL_SLAM = 1
    MANUAL_CONTROL = 2
    MAPPING = 3
    NAVIGATION = 4

class DataCollectionState():
    IDLE = 0
    RAW_COLLECTION = 1
    RAW_PLAYBACK = 2

class DeleteState():
    IDLE = 0
    DELETE = 1

class RobotHandler():
    operation_user_friendly_names = {'start_normal_slam': 'SLAM',
                                     'start_manual_control': 'manual control',
                                     'start_mapping': 'mapping',
                                     'save_map': 'save map',
                                     'start_nav': 'navigation'}

    operation_required_states = {'start_normal_slam': RobotState.IDLE,
                                 'start_manual_control': RobotState.IDLE,
                                 'start_mapping': RobotState.IDLE,
                                 'save_map': RobotState.MAPPING,
                                 'start_nav': RobotState.IDLE}

    operation_new_states = {'start_normal_slam': RobotState.NORMAL_SLAM,
                            'start_manual_control': RobotState.MANUAL_CONTROL,
                            'start_mapping': RobotState.MAPPING,
                            'save_map': RobotState.MAPPING,
                            'start_nav': RobotState.NAVIGATION}

    operation_launch_params = {'start_normal_slam': ['turn_on_wheeltec_robot', 'rrt_slam.launch', 'control'],
                               'start_manual_control': ['turn_on_wheeltec_robot', 'turn_on_wheeltec_robot.launch', 'control'],
                               'start_mapping': ['turn_on_wheeltec_robot', 'mapping.launch', 'control'],
                               'save_map': ['turn_on_wheeltec_robot', 'map_saver.launch', 'save_map'],
                               'start_nav': ['turn_on_wheeltec_robot', 'navigation.launch', 'control']}

    def __init__(self):
        self.currentState = RobotState.IDLE
        self.currentCollectionState = DataCollectionState.IDLE

        self.proc_manager = SubProcessManager()

        self.delete_state = DeleteState.IDLE

        # Register publishers
        self.robotHandlerStatusPub = rospy.Publisher('robot_handler_status', String, queue_size=10)
        self.capListPub = rospy.Publisher('robot_handler_cap_file_list', String, queue_size=10)

        # Publisher used later for the halt button
        self.halt_goal_publisher = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)

    def run_launch_file(self, package, launch_file, args='', mode='control'):
        success = True
        try:
            self.robotHandlerStatusPub.publish('start ' + launch_file)

            self.proc_manager.create_new_subprocess(mode, 'roslaunch ' + package + ' ' + launch_file + ' ' + args)

        except:
            self.robotHandlerStatusPub.publish('exception occurred running ' + launch_file)
            success = False

        return success

    def start_operation(self, command):
        roslaunch_params = self.operation_launch_params[command]
        friendly_name = self.operation_user_friendly_names[command]
        if self.currentState == self.operation_required_states[command]:
            success = self.run_launch_file(roslaunch_params[0], roslaunch_params[1], mode=roslaunch_params[2])
            if success:
                self.currentState = self.operation_new_states[command]
                self.robotHandlerStatusPub.publish('successfully started ' + friendly_name)
        else:
            self.robotHandlerStatusPub.publish('robot must be idle to start ' + friendly_name + '!')

    def stop_all(self):
        if self.currentState != RobotState.IDLE and self.proc_manager.is_subprocess_running('control'):
            self.proc_manager.close_subprocess('control')
            self.currentState = RobotState.IDLE

        self.robotHandlerStatusPub.publish('STOP')
    
    def halt(self):
        self.halt_goal_publisher.publish(GoalID())
        self.robotHandlerStatusPub.publish('Halting Rover')

    def set_max_speed(self, max_speed):
        self.robotHandlerStatusPub.publish('max speed set at ' + max_speed)

    def toggle_collection(self, device):
        if self.currentCollectionState == DataCollectionState.IDLE:
            success = self.run_launch_file('data_collection', device + '_collection.launch', mode="data_collection")
            if success:
                self.currentCollectionState = DataCollectionState.RAW_COLLECTION
                self.robotHandlerStatusPub.publish('successfully started data collection for lidar')
        elif self.currentCollectionState == DataCollectionState.RAW_COLLECTION:
            if self.currentCollectionState != DataCollectionState.IDLE and \
                    self.proc_manager.is_subprocess_running('data_collection'):
                self.proc_manager.close_subprocess('data_collection')
                self.currentCollectionState = DataCollectionState.IDLE

                self.robotHandlerStatusPub.publish('Stop data collection')
        else:
            self.robotHandlerStatusPub.publish('There must be no data playback active to toggle capturing')

        self.send_data_cap_list(device)

    def toggle_playback(self, device, file_name):
        if self.currentCollectionState == DataCollectionState.IDLE:
            success = self.run_launch_file('data_collection',  'playback_data.launch',
                                           args='file_name:=' + file_name + ' device_name:=' + device,
                                           mode="data_collection")
            if success:
                self.currentCollectionState = DataCollectionState.RAW_PLAYBACK
                self.robotHandlerStatusPub.publish('successfully started data playback for lidar')
        elif self.currentCollectionState == DataCollectionState.RAW_PLAYBACK:
            if self.currentCollectionState != DataCollectionState.IDLE and \
                    self.proc_manager.is_subprocess_running('data_collection'):
                self.proc_manager.close_subprocess('data_collection')
                self.currentCollectionState = DataCollectionState.IDLE

            self.robotHandlerStatusPub.publish('Stop data playback')
        else:
            self.robotHandlerStatusPub.publish('There must be no data collection active to toggle playback')

    def send_data_cap_list(self, device):
        files_message = ''
        files = os.listdir(ROBOT_CAP_SAVE_DIRECTORY + device + '/')

        for file in files:
            files_message += file + "|"

        self.capListPub.publish(files_message)

    def del_data_cap(self, device, file_name):
        if self.proc_manager.create_new_subprocess('delete_data_cap', 'rm ' + ROBOT_CAP_SAVE_DIRECTORY + device + '/' +
                                                    file_name):
            self.robotHandlerStatusPub.publish('Start deletion of data cap')
            self.delete_state == DeleteState.DELETE
            self.send_data_cap_list(device)
        else:
            self.robotHandlerStatusPub.publish('Data cap already being deleted')

    def update(self):
        if not self.proc_manager.is_subprocess_running('delete_data_cap') and self.delete_state == DeleteState.DELETE:
            self.robotHandlerStatusPub.publish('Data cap deletion complete')
            self.delete_state = DeleteState.IDLE
        if not self.proc_manager.is_subprocess_running('data_collection') and self.currentCollectionState != \
                DataCollectionState.IDLE:
            self.robotHandlerStatusPub.publish('Data collection task complete')
            self.currentCollectionState = DataCollectionState.IDLE

if __name__ == '__main__':
    def main():
        try:
            while not rospy.is_shutdown():
                robot_handler.update()

        except KeyboardInterrupt:
            print("Node shutting down due to keyboard interrupt signal.")

    def command_handler(cmd):
        # Split command
        cmd = cmd.data.split(" ")

        # Data collection
        if cmd[0] == 'toggle_collection':
            robot_handler.toggle_collection(cmd[1])
        elif cmd[0] == 'get_cap_file_list':
            robot_handler.send_data_cap_list(cmd[1])
        elif cmd[0] == 'toggle_playback':
            robot_handler.toggle_playback(cmd[1], cmd[2])
        elif cmd[0] == 'delete_data_cap':
            robot_handler.del_data_cap(cmd[1], cmd[2])

        # Robot operations
        elif cmd[0] == 'stop_all':
            robot_handler.stop_all()
        elif cmd[0] == 'halt':
            robot_handler.halt()
        elif cmd[0] == 'set_max_speed':
            robot_handler.set_max_speed(cmd[1])
        else:
            robot_handler.start_operation(cmd[0])

    robot_handler = RobotHandler()

    node_name = 'robot_handler'
    rospy.init_node(node_name, anonymous=False)  # only allow one node of this type
    rospy.loginfo('Initialized "' + node_name + '" node for pub/sub/service functionality')

    command_topic = '/robot_handler_cmd'
    rospy.loginfo('Beginning to subscribe to "' + command_topic + '" topic')
    sub = rospy.Subscriber(command_topic, String, command_handler)

    main()
