#!/usr/bin/env python

import rospy
import roslaunch

from std_msgs.msg import String

class RobotState():
    IDLE = 0
    MANUAL_CONTROL = 1
    MAPPING = 2
    NAVIGATION = 3
    MAPPING_AND_NAVIGATION = 4 # currently unused

class RobotHandler():
    def __init__(self):
        self.currentState = RobotState.IDLE

        self.control_thread = None
        self.save_map_thread = None
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        # Register publishers
        self.robotHandlerCommandPub = rospy.Publisher('robot_handler_status', String, queue_size=10)

        # Python doesn't like signals not being created in its main process, which some nodes under this launch file
        # attempts to do in separate threads. We don't need to handle these signals ourselves here, so this resolves it.
        # NOTE: this is pretty cursed but works for now lol
        def dummy_function(): pass
        roslaunch.pmon._init_signal_handlers = dummy_function

    def run_launch_file(self, package, launch_file, is_saving_map=False):
        success = True
        try:
            self.robotHandlerCommandPub.publish('start ' + launch_file)

            ros_thread = \
                roslaunch.parent.ROSLaunchParent(self.uuid,
                                                 roslaunch.rlutil.resolve_launch_arguments([package, launch_file]))

            if is_saving_map:
                self.save_map_thread = ros_thread
                self.save_map_thread.start()
            else:
                self.control_thread = ros_thread
                self.control_thread.start()

        except():
            self.robotHandlerCommandPub.publish('exception occurred running ' + launch_file)
            success = False

        return success

    def start_manual_control(self):
        if self.currentState == RobotState.IDLE:
            success = self.run_launch_file('turn_on_wheeltec_robot', 'turn_on_wheeltec_robot.launch')
            if success:
                self.currentState = RobotState.MAPPING
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
            success = self.run_launch_file('turn_on_wheeltec_robot', 'map_saver.launch', True)
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

if __name__ == '__main__':
    def main():
        rospy.spin()

    def command_handler(cmd):
        if cmd.data == 'start_manual_control':
            robot_handler.start_manual_control()
        elif cmd.data == 'start_mapping':
            robot_handler.start_mapping()
        elif cmd.data == 'save_map':
            robot_handler.save_map()
        elif cmd.data == 'start_nav':
            robot_handler.start_navigation()
        elif cmd.data == 'stop_all':
            robot_handler.stop_all()

    robot_handler = RobotHandler()

    node_name = 'robot_handler'
    rospy.init_node(node_name, anonymous=False)  # only allow one node of this type
    rospy.loginfo('Initialized "' + node_name + '" node for pub/sub/service functionality')

    command_topic = '/robot_handler_cmd'
    rospy.loginfo('Beginning to subscribe to "' + command_topic + '" topic')
    sub = rospy.Subscriber(command_topic, String, command_handler)

    main()
