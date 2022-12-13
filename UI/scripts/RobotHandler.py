#!/usr/bin/env python

import rospy
import roslaunch

from std_msgs.msg import String

class RobotState():
    IDLE = 0
    NORMAL_SLAM = 1
    MANUAL_CONTROL = 2
    MAPPING = 3
    NAVIGATION = 4

class DataCollectionState():
    IDLE = 0
    RAW_LIDAR_COLLECTION = 1

class RobotHandler():
    def __init__(self):
        self.currentState = RobotState.IDLE
        self.currentCollectionState = DataCollectionState.IDLE

        # ROS launch thread control
        self.control_thread = None
        self.save_map_thread = None
        self.data_collection_thread = None
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        # Register publishers
        self.robotHandlerCommandPub = rospy.Publisher('robot_handler_status', String, queue_size=10)

        # Python doesn't like signals not being created in its main process, which some nodes under this launch file
        # attempts to do in separate threads. We don't need to handle these signals ourselves here, so this resolves it.
        # NOTE: this is pretty cursed but works for now lol
        def dummy_function(): pass
        roslaunch.pmon._init_signal_handlers = dummy_function

    def run_launch_file(self, package, launch_file, mode='control'):
        success = True
        try:
            self.robotHandlerCommandPub.publish('start ' + launch_file)

            ros_thread = \
                roslaunch.parent.ROSLaunchParent(self.uuid,
                                                 roslaunch.rlutil.resolve_launch_arguments([package, launch_file]))

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
            self.robotHandlerCommandPub.publish('exception occurred running ' + launch_file)
            success = False

        return success
    def start_normal_slam(self):
        if self.currentState == RobotState.IDLE:
            success = self.run_launch_file('turn_on_wheeltec_robot', 'rrt_slam.launch')
            if success:
                self.currentState = RobotState.NORMAL_SLAM
                self.robotHandlerCommandPub.publish('successfully started SLAM')
        else:
            self.robotHandlerCommandPub.publish('robot must be idle to start SLAM!')
    def start_manual_control(self):
        if self.currentState == RobotState.IDLE:
            success = self.run_launch_file('turn_on_wheeltec_robot', 'turn_on_wheeltec_robot.launch')
            if success:
                self.currentState = RobotState.MANUAL_CONTROL
                self.robotHandlerCommandPub.publish('successfully started manual control')
        else:
            self.robotHandlerCommandPub.publish('robot must be idle to start manual teleop!')

    def start_mapping(self):
        if self.currentState == RobotState.IDLE:
            success = self.run_launch_file('turn_on_wheeltec_robot', 'mapping.launch')
            if success:
                self.currentState = RobotState.MAPPING
                self.robotHandlerCommandPub.publish('successfully started mapping')
        else:
            self.robotHandlerCommandPub.publish('robot must be idle to start mapping!')

    def save_map(self):
        if self.currentState == RobotState.MAPPING:
            success = self.run_launch_file('turn_on_wheeltec_robot', 'map_saver.launch', "save_map")
            if success:
                self.robotHandlerCommandPub.publish('successfully saved map')
        else:
            self.robotHandlerCommandPub.publish('Mapping must be running in order to save map!')

    def start_navigation(self):
        if self.currentState == RobotState.IDLE:
            success = self.run_launch_file('turn_on_wheeltec_robot', 'navigation.launch')
            if success:
                self.currentState = RobotState.NAVIGATION
                self.robotHandlerCommandPub.publish('successfully started navigation')
        else:
            self.robotHandlerCommandPub.publish('robot must be idle to start navigation!')

    def stop_all(self):
        if self.currentState != RobotState.IDLE and self.control_thread is not None:
            self.control_thread.shutdown()
            self.currentState = RobotState.IDLE

        self.robotHandlerCommandPub.publish('STOP')

    def start_lidar_collection(self):
        if self.currentCollectionState == DataCollectionState.IDLE:
            success = self.run_launch_file('data_collection', 'lidar_collection.launch', "data_collection")
            if success:
                self.currentCollectionState = DataCollectionState.RAW_LIDAR_COLLECTION
                self.robotHandlerCommandPub.publish('successfully started data collection for lidar')
        else:
            self.robotHandlerCommandPub.publish('There must be no other collection and lidar data must be publishing!')

    def stop_lidar_collection(self):
        if self.currentCollectionState != DataCollectionState.IDLE and self.data_collection_thread is not None:
            self.data_collection_thread.shutdown()
            self.currentCollectionState = RobotState.IDLE

        self.robotHandlerCommandPub.publish('Stop data collection')

if __name__ == '__main__':
    def main():
        rospy.spin()

    def command_handler(cmd):
        # Split command
        cmd = cmd.split(" ")

        # Robot operations
        if cmd[0].data == 'start_normal_slam':
            robot_handler.start_normal_slam()
        elif cmd[0].data == 'start_manual_control':
            robot_handler.start_manual_control()
        elif cmd[0].data == 'start_mapping':
            robot_handler.start_mapping()
        elif cmd[0].data == 'save_map':
            robot_handler.save_map()
        elif cmd[0].data == 'start_nav':
            robot_handler.start_navigation()
        elif cmd[0].data == 'stop_all':
            robot_handler.stop_all()

        # Data collection
        elif cmd[0].data == 'start_lidar_collection':
            robot_handler.start_lidar_collection()
        elif cmd[0].data == 'stop_lidar_collection':
            robot_handler.stop_lidar_collection()

    robot_handler = RobotHandler()

    node_name = 'robot_handler'
    rospy.init_node(node_name, anonymous=False)  # only allow one node of this type
    rospy.loginfo('Initialized "' + node_name + '" node for pub/sub/service functionality')

    command_topic = '/robot_handler_cmd'
    rospy.loginfo('Beginning to subscribe to "' + command_topic + '" topic')
    sub = rospy.Subscriber(command_topic, String, command_handler)

    main()
