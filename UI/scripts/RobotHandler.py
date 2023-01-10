#!/usr/bin/env python

import os

import rospy
import roslaunch

from std_msgs.msg import String

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

        # ROS launch thread control
        self.control_thread = None
        self.save_map_thread = None
        self.data_collection_thread = None
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        self.proc_manager = SubProcessManager()

        # Register publishers
        self.robotHandlerStatusPub = rospy.Publisher('robot_handler_status', String, queue_size=10)
        self.capListPub = rospy.Publisher('robot_handler_cap_file_list', String, queue_size=10)

        # Python doesn't like signals not being created in its main process, which some nodes under this launch file
        # attempts to do in separate threads. We don't need to handle these signals ourselves here, so this resolves it.
        # NOTE: this is pretty cursed but works for now lol
        def dummy_function(): pass
        roslaunch.pmon._init_signal_handlers = dummy_function

    def run_launch_file(self, package, launch_file, args=None, mode='control'):
        success = True
        try:
            self.robotHandlerStatusPub.publish('start ' + launch_file)

            ros_thread = \
                roslaunch.parent.ROSLaunchParent(self.uuid,
                                                 roslaunch.rlutil.resolve_launch_arguments(
                                                     [package, ([launch_file], args)]))

            if mode == 'control':
                self.control_thread = ros_thread
                self.control_thread.start()
            elif mode == 'save_map':
                self.save_map_thread = ros_thread
                self.save_map_thread.start()
            elif mode == 'data_collection':
                self.data_collection_thread = ros_thread
                self.data_collection_thread.start()


        except():
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
        if self.currentState != RobotState.IDLE and self.control_thread is not None:
            self.control_thread.shutdown()
            self.currentState = RobotState.IDLE

        self.robotHandlerStatusPub.publish('STOP')

    def toggle_collection(self, device):
        if self.currentCollectionState == DataCollectionState.IDLE:
            success = self.run_launch_file('data_collection', device + '_collection.launch', mode="data_collection")
            if success:
                self.currentCollectionState = DataCollectionState.RAW_COLLECTION
                self.robotHandlerStatusPub.publish('successfully started data collection for lidar')
        elif self.currentCollectionState == DataCollectionState.RAW_COLLECTION:
            if self.currentCollectionState != DataCollectionState.IDLE and self.data_collection_thread is not None:
                self.data_collection_thread.shutdown()
                self.currentCollectionState = DataCollectionState.IDLE

                self.robotHandlerStatusPub.publish('Stop data collection')
        else:
            self.robotHandlerStatusPub.publish('There must be no data playback active to toggle capturing')

        self.send_data_cap_list(device)

    def toggle_playback(self, device, file_name):
        if self.currentCollectionState == DataCollectionState.IDLE:
            success = self.run_launch_file('data_collection',  'playback_data.launch',
                                           args=['file_name:=' + file_name, 'device_name:=' + device],
                                           mode="data_collection")
            if success:
                self.currentCollectionState = DataCollectionState.RAW_PLAYBACK
                self.robotHandlerStatusPub.publish('successfully started data playback for lidar')
        elif self.currentCollectionState == DataCollectionState.RAW_PLAYBACK:
            if self.currentCollectionState != DataCollectionState.IDLE and self.data_collection_thread is not None:
                self.data_collection_thread.shutdown()
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
        else:
            self.robotHandlerStatusPub.publish('Data cap already being deleted')

    def update(self):
        self.proc_manager.update()
        if self.proc_manageris_subprocess_running('delete_data_cap'):
            self.capListPub.publish('Data cap deletion complete')

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

        # Robot operations
        elif cmd[0] == 'stop_all':
            robot_handler.stop_all()
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
