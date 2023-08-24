import os
import pickle

import cv2
import rospy
import rospkg
import rosbag

import subprocess
import re
import time
import math

import datetime
import threading

import webbrowser

from sensor_msgs.msg import Imu
from std_msgs.msg import String, Header, Float32
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, Vector3
from move_base_msgs.msg import MoveBaseActionResult
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from bodyreader.msg import bodyposture

from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QDialog, QScrollArea

from PyQt5.QtWidgets import QPushButton, QListWidget, QComboBox, QDoubleSpinBox, QShortcut, QRadioButton, QSlider, QLabel, QCheckBox, QLineEdit, QLCDNumber, QTableWidget, QProgressBar, QSpinBox, QTableWidgetItem, QSizePolicy
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QObject, Qt, qRegisterResourceData
from PyQt5.QtGui import QColor, QImage, QPixmap


from cv_bridge import CvBridge, CvBridgeError

from PIL import Image as PIM
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from utils.Definitions import *
from utils.SubProcessManager import *
from point_input import Point_input


COORDINATE_FILE = "/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/src/wheeltec-building-inspection/possible_targets.pickle"

class TransferState():
    IDLE = 0
    TRANSFER = 1

class NavControl(Plugin):
    def __init__(self, context):
        super(NavControl, self).__init__(context)

        self.proc_manager = SubProcessManager()
        self.transfer_state = TransferState.IDLE


        # Give QObjects reasonable names
        self.setObjectName('NavControl')

        # Register publishers
        self.robotHandlerCommandPub = rospy.Publisher('robot_handler_cmd', String, queue_size=10)
        self.velocityCommandPub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.pointClickedCommandPub = rospy.Publisher('clicked_point', PointStamped, queue_size=10)

        # Register subscribers
        self.robotHandlerStatusSub = rospy.Subscriber('robot_handler_status', String, self.add_to_log_console)
        self.robotHandlerCapListRequest = rospy.Subscriber('robot_handler_cap_file_list', String, self.refresh_robot_cap_list)
        self.robotHandlerCommandSub = rospy.Subscriber('robot_handler_cmd', String, self.handle_commands)

        # Create QWidget
        self.scroll_area = QScrollArea()
        self.content_widget = QWidget()
        self._widget = QMainWindow()
        ui_file = os.path.join(rospkg.RosPack().get_path('wheeltec-building-inspection-ui'), 'resource',
                               'building-inspec-control.ui')
        loadUi(ui_file, self.content_widget)
        self._widget.setObjectName('NavControlUi')
        self.scroll_area.setWidget(self.content_widget)
        self._widget.setCentralWidget(self.scroll_area)

        self.path_planner_window = QDialog()
        self.path_planner_ui_file = os.path.join(rospkg.RosPack().get_path('wheeltec-building-inspection-ui'), 'resource',
                        'path-planner.ui')
        self.path_plan_ui = loadUi(self.path_planner_ui_file, self.path_planner_window)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # TODO: Use loop to create handlers
        # Has some weird behavior with all buttons inheriting the last event in this list
        # controlCommands = [{'button_name': 'StartNormalSlam', 'command': 'start_normal_slam'},
        #                    {'button_name': 'StartMapping', 'command': 'start_mapping'},
        #                    {'button_name': 'SaveMap', 'command': 'save_map'},
        #                    {'button_name': 'StartNav', 'command': 'start_nav'},
        #                    {'button_name': 'StopAll', 'command':'stop_all'}]

        # Event handlers for buttons
        # NavControl Tab
        # self._widget.findChild(QPushButton, 'StartManualControl').clicked.connect(
        #     lambda: self.robotHandlerCommandPub.publish('start_manual_control'))

        
        self._widget.findChild(QSlider, 'linear_velocity_slider').valueChanged.connect(
            self.update_velocity
        )

        self._widget.findChild(QSlider, 'angular_velocity_slider').valueChanged.connect(
            self.update_velocity
        )

        self._widget.findChild(QPushButton, 'reset_linear_velocity_button').clicked.connect(
            lambda: self._widget.findChild(QSlider, 'linear_velocity_slider').setValue(0)
        )

        self._widget.findChild(QPushButton, 'reset_angular_velocity_button').clicked.connect(
            lambda: self._widget.findChild(QSlider, 'angular_velocity_slider').setValue(0)
        )

        self._widget.findChild(QPushButton, 'increase_linear_velocity_button').clicked.connect(
            lambda: self.update_velocity_slider("linear", "+")
        )

        self._widget.findChild(QPushButton, 'decrease_linear_velocity_button').clicked.connect(
            lambda: self.update_velocity_slider("linear", "-")
        )

        self._widget.findChild(QPushButton, 'increase_angular_velocity_button').clicked.connect(
            lambda: self.update_velocity_slider("angular", "+")
        )

        self._widget.findChild(QPushButton, 'decrease_angular_velocity_button').clicked.connect(
            lambda: self.update_velocity_slider("angular", "-")
        )

        self._widget.findChild(QPushButton, 'StartNormalSlam').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('start_normal_slam')
        )

        self._widget.findChild(QPushButton, 'StartMapping').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('start_mapping'))

        self._widget.findChild(QPushButton, 'SaveMap').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('save_map'))

        self._widget.findChild(QPushButton, 'StartNav').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('start_nav'))
        
        self.x_coord = self._widget.findChild(QLineEdit, 'coordinate_x_output')
        self.y_coord = self._widget.findChild(QLineEdit, 'coordinate_y_output')

       
        self._widget.findChild(QPushButton, 'save_coordinates_button').pressed.connect(
            lambda: self.save_coordinates([float(self.x_coord.text()), float(self.y_coord.text())], name=(str(rospy.get_time())))
        )
        
        # Halts the rover's movement
        self.halt_sequence = QShortcut("Space", self._widget)
        self.halt_sequence.activated.connect(
            self.reset_velocities
        )

        # # Kills the current Launch file on board of the rover
        # self.estop_sequence = QShortcut("K", self._widget)
        # self.estop_sequence.activated.connect(
        #     lambda: self.robotHandlerCommandPub.publish('stop_all')
        # )

        # Data Cap Tab
        self.device_selection = self._widget.findChild(QComboBox, 'selectDeviceBox')
        self.device_selection.addItem('lidar')
        # TODO: add data cap for camera
        # self.device_selection.addItem('camera')

        self.fileCapView = self._widget.findChild(QListWidget, 'ListCaptureFiles')
        self.robotHandlerCommandPub.publish('get_cap_file_list ' + self.device_selection.currentText())

        self._widget.findChild(QPushButton, 'ToggleCollectionButton').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('toggle_collection ' + self.device_selection.currentText()))

        self._widget.findChild(QPushButton, 'TogglePlaybackButton').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('toggle_playback ' + self.device_selection.currentText() + ' '
                                                        + (self.fileCapView.currentItem().text() if
                                                           self.fileCapView.currentItem() is not None else 'None')))

        self._widget.findChild(QPushButton, 'DelCapButton').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('delete_data_cap ' + self.device_selection.currentText() + ' '
                                                        + (self.fileCapView.currentItem().text() if
                                                           self.fileCapView.currentItem() is not None else 'None')))

        self._widget.findChild(QPushButton, 'TransferCapButton').clicked.connect(
            lambda: self.transfer_cap_file())
        
        self.camera_button = self._widget.findChild(QRadioButton, 'EnableCamButton')
        self.camera_button.toggled.connect(
            lambda: self.robotHandlerCommandPub.publish('start_cam ' + str(int(self.camera_button.isChecked())))
        )
        
        #Enable Keyboard for controller to start
        self._widget.findChild(QPushButton, 'EnableKeyboard').clicked.connect(self.keyboard_on)
        #Load Map
        self._widget.findChild(QPushButton, 'LoadMap').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('load_map')
        )
        
        self._widget.findChild(QPushButton, 'SetPoints').clicked.connect(self.set_all_points)

        #New thread created to always check the signal strength and power voltage
        self.start_thread()
        

                
        self._widget.findChild(QPushButton, 'open_new_window_object').clicked.connect(self.open_new_window_object_detection)
        self._widget.findChild(QPushButton, 'open_new_window_skeleton').clicked.connect(self.open_new_window_skeleton_detection)
        self._widget.findChild(QPushButton, 'open_cam_browser').clicked.connect(self.open_browser)
        self._widget.findChild(QPushButton, 'visual_follower').clicked.connect(self.visual_follower)
        # self._widget.findChild(QPushButton, 'robot_follower').clicked.connect(self.robot_follower)
        self._widget.findChild(QPushButton, 'robot_follower').hide()
        self._widget.findChild(QPushButton, 'show_parameters_button').clicked.connect(self.open_new_window_param_list)
        self.hide_button_state = True
        self.hide_button = self._widget.findChild(QPushButton, 'hide_signal_strength').clicked.connect(self.hide_signal_strength)
        self._widget.findChild(QPushButton, 'open_recorder_window').clicked.connect(lambda: self.open_recorder_window(context))
        self._widget.findChild(QPushButton, 'movement_control').clicked.connect(self.open_movement_control_window)
        self._widget.findChild(QPushButton, 'kill_lidar').clicked.connect(self.kill_lidar)
        self._widget.findChild(QPushButton, 'show_coordinates').clicked.connect(self.run_tf_echo)
        self._widget.findChild(QPushButton, 'turn_off_coordinates').clicked.connect(self.turn_off_gps)
        self._widget.findChild(QPushButton, 'task_planning_go').clicked.connect(self.task_planning_go)
        self._widget.findChild(QPushButton, 'spin_360_button').clicked.connect(self.spin)
        self.lidar_on = self._widget.findChild(QPushButton, 'turn_on_lidar')
        self.lidar_on.setEnabled(False)
        self.lidar_on.clicked.connect(self.turn_on_lidar)

        self._widget.findChild(QPushButton, 'path_planner_button').clicked.connect(self.open_path_planner)

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        # General
        self._widget.findChild(QPushButton, 'StopAll').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('stop_all'))

        self._widget.findChild(QPushButton, 'Halt').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('halt 0'))
        
        self.halt_shortcut = QShortcut('k', self._widget)
        self.halt_shortcut.activated.connect(
            lambda: self.robotHandlerCommandPub.publish('halt 0'))

        

        self._widget.findChild(QDoubleSpinBox, 'max_linear_speed_input').editingFinished.connect(
            lambda: self.set_max_slider("linear")
        )
        self._widget.findChild(QDoubleSpinBox, 'max_angular_speed_input').editingFinished.connect(
            lambda: self.set_max_slider("angular")
        )

        # Handle log console
        self.log_console = self._widget.findChild(QListWidget, 'LogConsole')

        # TODO: Add update method for checking status of subprocesses using QTimer object
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start()

        self.rec_window = QWidget()
        self.goal_reached = True
        self.queue_state = 0
        self.demo_path = [
            [(2.07, -0.23, 0, 1), 0, 0], 
            [(3, 1, 0, 1), 0, 0], 
            [(4.5, 0.4, 1, 0), 0, 0], 
            [(2.4, 0.4, -0.72, 0.69), 0, 0]
        ]
        self.goal_list = []


    def spin_360(self, rotation_duration):
            start_time = rospy.Time.now()

            while rospy.Time.now() - start_time <= rotation_duration:

                msg = Twist()
                msg.linear.x = 0
                msg.linear.y = 0
                msg.linear.z = 0
                msg.angular.x = 0
                msg.angular.y = 0
                msg.angular.z = self._widget.findChild(QDoubleSpinBox, 'max_angular_speed_input').value()
                self.velocityCommandPub.publish(msg)
                rospy.sleep(0.1)
#lol
            stop_msg = Twist()
            self.velocityCommandPub.publish(stop_msg)

    def spin(self, angle):
        recording_duration = angle / self._widget.findChild(QDoubleSpinBox, 'max_angular_speed_input').value()
        thread1 = threading.Thread(target=self.spin_360, args=(rospy.Duration.from_sec(recording_duration),))
        thread1.start()
        time.sleep(recording_duration)

    def capture_image(self):
        save_path = "saved_images/image_" + str(rospy.get_time()) + ".jpg"
        topic = "/camera/rgb/image_raw"
        
        def image_callback(msg):
            try:
                bridge = CvBridge()
                # Convert ROS Image message to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imwrite(save_path, cv_image)
                print("Image saved to " + save_path)
                self.robotHandlerCommandPub.publish(String("Image saved to " + save_path))
            except CvBridgeError as e:
                print(e)

        subscriber = rospy.Subscriber(topic, Image, image_callback)
        rospy.wait_for_message(topic, Image)
        subscriber.unregister()


    #task planning
    def task_planning_go(self, goal_list = []):
        # if True:
        #     rospy.loginfo("Navigation should be enabled")
        #     return

        def send_new_nav_goal(goal_list):
            for goal in goal_list:
                if self.goal_reached == True:
                    # x, y, z, w = 0, 0, 0, 0
                    # if goal[0][0]:
                    #     x = goal[0][0]
                    # if goal[0][1]:
                    #     y = goal[0][1]
                    # if goal[0][2]:
                    #     z = goal[0][2]
                    # if goal[0][3]:
                    #     w = goal[0][3]

                    x, y, z, w = goal[0]

                    wait_time = goal[1]
                    angle = goal[2]
                    take_picture = goal[3]

                    rospy.loginfo("Sending goal to: " + "(" + str(x) + " , " + str(y) + ")")

                    goal_msg = PoseStamped()
                    goal_msg.header = Header()
                    goal_msg.header.stamp = rospy.Time.now()


                    goal_msg.pose.position.x = x
                    goal_msg.pose.position.y = y
                    goal_msg.pose.orientation.z = z
                    goal_msg.pose.orientation.w = w
                    goal_msg.header.frame_id = "map"

                    self.goal_pub.publish(goal_msg)
                    rospy.loginfo("Navigation goal sent!")
                    self.goal_reached = False
                    while not self.goal_reached:
                        pass
                    if angle:
                        self.spin(angle)
                    if take_picture:
                        self.capture_image()
                    if wait_time:
                        rospy.sleep(wait_time)
                        print("Wait Time Over")
            # while self.queue_state < len(goal_list):
            #     if self.goal_reached == True:
                
            #         if self.queue_state < len(goal_list):
                        # x, y, z, w = 0, 0, 0, 0
                        # if goal_list[self.queue_state][0][0]:
                        #     x = goal_list[self.queue_state][0][0]
                        # if goal_list[self.queue_state][0][1]:
                        #     y = goal_list[self.queue_state][0][1]
                        # if goal_list[self.queue_state][0][2]:
                        #     z = goal_list[self.queue_state][0][2]
                        # if goal_list[self.queue_state][0][3]:
                        #     w = goal_list[self.queue_state][0][3]

                        # wait_time = goal_list[self.queue_state][1]
                        # angle = goal_list[self.queue_state][2]
                        # take_picture = goal_list[self.queue_state][3]

                        # rospy.loginfo("Sending goal to: " + "(" + str(x) + " , " + str(y) + ")")

                        # goal_msg = PoseStamped()
                        # goal_msg.header = Header()
                        # goal_msg.header.stamp = rospy.Time.now()


                        # goal_msg.pose.position.x = x
                        # goal_msg.pose.position.y = y
                        # goal_msg.pose.orientation.z = z
                        # goal_msg.pose.orientation.w = w
                        # goal_msg.header.frame_id = "map"

                        # self.goal_pub.publish(goal_msg)
                        # rospy.loginfo("Navigation goal sent!")
                        # self.goal_reached = False
                        # while not self.goal_reached:
                        #     pass
                        # if angle:
                        #     self.spin(angle)
                        # if take_picture:
                        #     self.capture_image()
                        # if wait_time:
                        #     rospy.sleep(wait_time)
                        #     print("Wait Time Over")

        def goal_result_callback(msg):
            if msg.status.status == 3:
                print("REACHED")
                if self.goal_reached == False:
                    self.goal_reached = True
                    print("Goal is True")
                    self.queue_state += 1

            
        try:
            self.queue_state = 0
            rospy.Subscriber('/move_base/result', MoveBaseActionResult, goal_result_callback)

            if not goal_list:
                goal_list = self.demo_path

            def goal():
                send_new_nav_goal(goal_list)

            thread1 = threading.Thread(target=goal)
            thread1.start()
                

        except rospy.ROSInterruptException:
            pass
        # self.queue_state = 0

    def turn_off_gps(self):
        try:
            subprocess.call(['rosnode', 'kill', '/tf_echo'])
        except:
            pass

    def run_tf_echo(self):
        def threadd():
            try:
                subprocess.call(['rosrun', 'wheeltec-building-inspection-ui', 'tf_echo', '/map', '/base_link'])
            except:
                print("Exception")
            
        try:
            thread1 = threading.Thread(target=threadd, args=())
            thread1.start()
            #subprocess.call(['rosrun', 'wheeltec-building-inspection-ui', 'tf_echo', '/map', '/base_link'])
        except:
            print("Exception")

    def kill_lidar(self):
        self.lidar_on.setEnabled(True)
        try:
            subprocess.call(['rosnode', 'kill', '/lslidar_driver_node'])
            print("Killing the Lidar Node")
        except:
            print("Error Occured while turning off the Lidar")


    def open_movement_control_window(self):
        try:
            self.holonomic_state = rospy.get_param('/move_base/TebLocalPlannerROS/global_plan_overwrite_orientation')
        except:
            self.holonomic_state = False
            

        def overwrite_orientation():
            if self.holonomic_state == True:
                print("Unchecked")
                subprocess.call(['rosrun', 'dynamic_reconfigure', 'dynparam', 'set', '/move_base/TebLocalPlannerROS', 'global_plan_overwrite_orientation', 'false'])
                self.holonomic_state = False
                print("Global Plan Overwrite Orientation is Set to False")
            else:
                print("Checked")
                subprocess.call(['rosrun', 'dynamic_reconfigure', 'dynparam', 'set', '/move_base/TebLocalPlannerROS', 'global_plan_overwrite_orientation', 'true'])
                self.holonomic_state = True
                print("Global Plan Overwrite Orientation is Set to True")


        def param_change():
            try:
                subprocess.call(['rosrun', 'dynamic_reconfigure', 'dynparam', 'set', '/move_base/TebLocalPlannerROS', 'weight_kinematics_nh', str(kinematics_box.value())])
                subprocess.call(['rosrun', 'dynamic_reconfigure', 'dynparam', 'set', '/move_base/TebLocalPlannerROS', 'acc_lim_y', '0.5'])
                subprocess.call(['rosrun', 'dynamic_reconfigure', 'dynparam', 'set', '/move_base/TebLocalPlannerROS', 'max_vel_y', '0.5'])
            except:
                print("ERRRRRRORRRRRR")

        def activate_holonomic_movement():
            threads = threading.Thread(target=param_change, args=())
            threads.start()

            print("Robot is now Holonomic")
            holonomic_label.setVisible(True)
        new_window_3 = QDialog()
        loadUi("/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/resource/movement-control-display.ui", new_window_3)

        new_window_3.findChild(QPushButton, 'activate_holonomic_movement').clicked.connect(activate_holonomic_movement)
        kinematics_box = new_window_3.findChild(QSpinBox, 'kinematics_box')
        holonomic_label = new_window_3.findChild(QLabel, 'holonomic_on_off_label')
        holonomic_label.setVisible(False)
        
        overwrite_orientation_box = new_window_3.findChild(QCheckBox, 'overwrite_orientation')
        if self.holonomic_state == True:
            overwrite_orientation_box.setChecked(True)
        else:
            overwrite_orientation_box.setChecked(False)
        overwrite_orientation_box.stateChanged.connect(overwrite_orientation)


        new_window_3.exec_()




    def open_recorder_window(self, cont):

        def check_topic_active(topic_name):
            type_map = {
                '/imu_raw': Imu,
                '/point_cloud_raw': PointCloud2,
                '/darknet_ros': BoundingBoxes,
                'darknet_ros/detection_image': Image,
            }

            topic_type = type_map[topic_name]
            try:
                rospy.wait_for_message(topic_name, topic_type, timeout=2)
                print("Topic " + topic_name + " is active and receiving messages.")
                return True
            except rospy.exceptions.ROSException:
                print("Topic " + topic_name + " is not active or not receiving messages.")
                return False

        def record_pcl():
            th = threading.Thread(target=record_point_cloud, args=())
            th.start()
        def record_imu_thread():
            th_1 = threading.Thread(target=record_imu, args=())
            th_1.start()
        



        def record_imu():

            def  extract_imu_data_from_bag(input_bag_file, output_text_file):
                with rosbag.Bag(input_bag_file, 'r') as bag:
                    with open(output_text_file, 'w') as output_file:
                        for topic, msg, t in bag.read_messages(topics=['/imu_raw']):  # Replace '/imu_topic' with your actual IMU topic name
                            if topic == '/imu_raw' and msg._type == 'sensor_msgs/Imu':  # Make sure the message type is 'sensor_msgs/Imu'
                                imu_data = "Timestamp: {}, " \
                                        "Orientation (x, y, z, w): {}, {}, {}, {}, " \
                                        "Angular velocity (x, y, z): {}, {}, {}, " \
                                        "Linear acceleration (x, y, z): {}, {}, {}\n".format(
                                    msg.header.stamp,
                                    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
                                )
                                output_file.write(imu_data)

            def callback(data):
                if rospy.Time.now() - start_time >= recording_duration:
                    print("BAG CLOSING...")
                    subscribe_pcl.unregister()
                    bag.close()

                    input_bag_file = '/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/imu_recordings/IMU-RECORDING.bag'
                    output_text_file = '/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/imu_recordings/IMU-RECORDING.txt'
                    extract_imu_data_from_bag(input_bag_file, output_text_file)

                else:
                    print(rospy.Time.now())
                    bag.write('/imu_raw', data)



            is_topic_active = check_topic_active('/imu_raw')
            if is_topic_active:
                record_duration = self.rec_window.findChild(QSpinBox, 'record_duration').value()
                bag = rosbag.Bag('/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/imu_recordings/IMU-RECORDING.bag', 'w')
                recording_duration = rospy.Duration.from_sec(record_duration)
                start_time = rospy.Time.now()
                print("Start Time: " + str(start_time))
                subscribe_pcl = rospy.Subscriber('/imu_raw', Imu, callback)

            else:
                print("Topic /imu_raw is not active")

        def record_point_cloud():

            def extract_lidar_data_from_bag(input_bag_file, output_text_file):
                with rosbag.Bag(input_bag_file, 'r') as bag:
                    with open(output_text_file, 'w') as output_file:
                        for topic, msg, t in bag.read_messages(topics=['/point_cloud_raw']):
                            if topic == '/point_cloud_raw' and msg._type == 'sensor_msgs/PointCloud2':
                                for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                                    lidar_data = "X: {:.2f} Y: {:.2f} Z: {:.2f}\n".format(p[0], p[1], p[2])
                                    output_file.write(lidar_data)

            def callback(data):
                if rospy.Time.now() - start_time >= recording_duration:
                    print("BAG CLOSING...")
                    subscribe_pcl.unregister()
                    bag.close()

                    input_bag_file = '/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/Lidar_recordings/Lidar-RECORDING.bag'
                    output_text_file = '/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/Lidar_recordings/Lidar-RECORDING.txt'
                    extract_lidar_data_from_bag(input_bag_file, output_text_file)


                else:
                    print(rospy.Time.now())
                    bag.write('/point_cloud_raw', data)



            is_topic_active = check_topic_active('/point_cloud_raw')
            if is_topic_active:
                record_duration = self.rec_window.findChild(QSpinBox, 'record_duration').value()
                bag = rosbag.Bag('/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/Lidar_recordings/Lidar-RECORDING.bag', 'w')
                recording_duration = rospy.Duration.from_sec(record_duration)
                start_time = rospy.Time.now()
                print("Start Time: " + str(start_time))
                subscribe_pcl = rospy.Subscriber('/point_cloud_raw', PointCloud2, callback)
            else:
                print("Topic /point_cloud_raw is not active")
            

        def record_skeleton_data():

            list_1 = []
            def backcall(data):
                if rospy.Time.now() - start_time1 >= recording_duration:
                    print("Recording Stopping...")
                    with open('/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/camera_recordings/skeleton_detection/skeleton.txt', "w") as file:
                        for list1 in list_1:
                            file.write(list1 +"\n")
                    subscribe_skel.unregister()
                else:
                    print(rospy.Time.now())
                    list_1.append(str(data))
                        ##########################################
                        ##############################################
                        #############################################


            is_topic_active2 = True
            if is_topic_active2:
                record_duration1 = self.rec_window.findChild(QSpinBox, 'record_duration').value()
                recording_duration = rospy.Duration.from_sec(record_duration1)
                start_time1 = rospy.Time.now()
                print("Start Time: ") + str(start_time1)
                subscribe_skel = rospy.Subscriber('/body_posture', bodyposture, backcall)

        def record_video_thread():
            bag = rosbag.Bag('/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/camera_recordings/object_detection/VIDEO-GENERATED.bag', 'w')
            recording_duration = rospy.Duration.from_sec(self.rec_window.findChild(QSpinBox, 'record_duration').value())
            start_time = rospy.Time.now()
            print("Start Time: " + str(start_time))

            def callback(data):
                if rospy.Time.now() - start_time >= recording_duration:
                    # Close the rosbag file when the recording duration ends
                    bag.close()
                    print("BAG CLOSING....")
                    subscriber.unregister()
                    command_1 = ['python2.7', '/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/src/wheeltec-building-inspection/rosbag2video.py', '/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/camera_recordings/object_detection/VIDEO-GENERATED.bag']
                    subprocess.call(command_1)
                    
                else:
                    print(rospy.Time.now())
                    bag.write('/darknet_ros/detection_image', data)

            subscriber = rospy.Subscriber('/darknet_ros/detection_image', Image, callback)
            rospy.spin()

        def record_object_detected_video():
            path1 = "/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/camera_recordings/object_detection/VIDEO-GENERATED_darknet_ros_detection_image.mp4"
            try:
                if os.path.exists(path1):
                    print("OVERWRITING...")
                    os.remove(path1)

                is_active = check_topic_active('/darknet_ros/detection_image')
                if is_active:
                    my_thread = threading.Thread(target=record_video_thread)
                    my_thread.start()
            except:
                print("Topic /darknet_ros/detection_image is not active")

        def record_object_detected():


            def backcall(data):
                if rospy.Time.now() - start_time1 >= recording_duration:
                    print("Recording Stopping...")
                    with open('/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/camera_recordings/object_detection/detected_object.txt', "w") as file:
                        for object1 in object_set:
                            file.write(object1 +"\n")
                    subscribe_yolo.unregister()
                else:
                    print(rospy.Time.now())
                    for box in data.bounding_boxes:
                        detected_class = box.Class
                        detected_class1 = str(detected_class)

                        top_left_x = box.xmin
                        top_left_y = box.ymin
                        bottom_right_x = box.xmax
                        bottom_right_y = box.ymax
                        center_x = (top_left_x + bottom_right_x) / 2
                        center_y = (top_left_y + bottom_right_y) / 2
                        confidence = box.probability
                        x_coord = self._widget.findChild(QLineEdit, 'entry_x').text()
                        y_coord = self._widget.findChild(QLineEdit, 'entry_y').text()
                        object_set.append("Detected: "+detected_class1 + " at " + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S") + " at " + "X: " +str(center_x) + ", Y: " + str(center_y) + " with confidence " + str(confidence) + " while position of the ROBOT on the map is X: " + x_coord + " Y: " + y_coord)

            is_topic_active1 = check_topic_active("/darknet_ros/bounding_boxes")
            if is_topic_active1:
                record_duration1 = self.rec_window.findChild(QSpinBox, 'record_duration').value()
                object_set = []
                recording_duration = rospy.Duration.from_sec(record_duration1)
                start_time1 = rospy.Time.now()
                print("Start Time: ") + str(start_time1)
                subscribe_yolo = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, backcall)


        if self.rec_window.isVisible():
            print("Window is already open")
        
        else:
            loadUi('/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/resource/recorder-window.ui', self.rec_window)
            cont.add_widget(self.rec_window)
        
        self.rec_window.findChild(QPushButton, 'record_lidar_point_cloud').clicked.connect(record_pcl)
        self.rec_window.findChild(QPushButton, 'record_imu').clicked.connect(record_imu_thread)
        self.rec_window.findChild(QPushButton, 'record_objects').clicked.connect(record_object_detected)
        self.rec_window.findChild(QPushButton, 'record_skeleton').clicked.connect(record_skeleton_data)
        self.rec_window.findChild(QPushButton, 'record_object_detected_video').clicked.connect(record_object_detected_video)

    def turn_on_lidar(self):
        self.robotHandlerCommandPub.publish('turn_on_lidar')
        self.lidar_on.setEnabled(False)

        


    def hide_signal_strength(self):
        lcd1 = self._widget.findChild(QLCDNumber, 'WifiStrength')
        lcd2 = self._widget.findChild(QLCDNumber, 'WifiStrength_2')
        label1 = self._widget.findChild(QLabel, 'label_7')
        label2 = self._widget.findChild(QLabel, 'label_8')

        self.hide_button_state = not self.hide_button_state

        lcd1.setVisible(self.hide_button_state)
        lcd2.setVisible(self.hide_button_state)
        label1.setVisible(self.hide_button_state)
        label2.setVisible(self.hide_button_state)

    def open_new_window_param_list(self):

        def handle_param_change(item):
            try:
                param_name = param_table_widget.item(item.row(), 0).text()
                param_value_str = param_table_widget.item(item.row(), 1).text()
                
                param_value_type = rospy.get_param(param_name)

                # Convert the user-inputted value to the original data type
                try:
                    if isinstance(param_value_type, int):
                        param_value = int(param_value_str)
                    elif isinstance(param_value_type, float):
                        param_value = float(param_value_str)
                    elif isinstance(param_value_type, bool):
                        param_value = param_value_str.lower() in ['true', '1']
                    else:
                        param_value = param_value_str
                    rospy.set_param(param_name, param_value)
                except rospy.ROSException as e:
                    print("Failed to update parameter: ")
            except:
                print("No params")
                
        def refresh_parameters_list():
            param_table_widget.setRowCount(len(ros_param_list))
            param_table_widget.setColumnCount(2)

            for i, param_name in enumerate(ros_param_list):
                param_value = rospy.get_param(param_name)
                param_table_widget.setItem(i, 0, QTableWidgetItem(param_name))
                param_table_widget.setItem(i, 1, QTableWidgetItem(str(param_value)))

            param_table_widget.itemChanged.connect(handle_param_change)

        def filter_parameters(search_text):
            filtered_params = [param for param in ros_param_list if search_text.lower() in param.lower()]
            param_table_widget.setRowCount(len(filtered_params))
            for i, param_name in enumerate(filtered_params):
                param_value = rospy.get_param(param_name)

                param_table_widget.setItem(i, 0, QTableWidgetItem(param_name))
                param_table_widget.setItem(i, 1, QTableWidgetItem(str(param_value)))



        new_window_2 = QDialog()
        loadUi("/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/resource/ros-parameters-display.ui", new_window_2)

        show_param_button = new_window_2.findChild(QPushButton, 'show_param')
        show_param_button.clicked.connect(refresh_parameters_list)
        param_table_widget = new_window_2.findChild(QTableWidget, 'param_table_widget')
        search_bar = new_window_2.findChild(QLineEdit, 'param_search_bar')
        search_bar.setPlaceholderText("Search parameters...")
        search_bar.textChanged.connect(filter_parameters)
        ros_param_list = rospy.get_param_names()
        ros_param_list = [param for param in ros_param_list if not param.startswith('~')]
        new_window_2.exec_()


    def callback1(self, data):
        # data_list = [str(data)]
        linear_x = data.linear.x
        linear_y = data.linear.y
        linear_z = data.linear.z
        angular_x = data.angular.x
        angular_y = data.angular.y
        angular_z = data.angular.z
        list_1 = [str(linear_x), str(linear_y), str(linear_z), str(angular_x), str(angular_y), str(angular_z)]
        # Construct the command with the extracted values as arguments
        #command = ['sh', '/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/src/wheeltec-building-inspection/follower.sh']
        # subprocess.call(['python', '/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/src/wheeltec-building-inspection/follower_1.py', str(data)])
        subprocess.call(['sh', '/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/src/wheeltec-building-inspection/follower.sh'] + list_1)

    
    def robot_follower(self):
        def my_thread_function():
            rospy.Subscriber("/cmd_vel", Twist, self.callback1)
            rospy.spin()
        print('Hello')
        my_thread = threading.Thread(target=my_thread_function)
        my_thread.start()
    



    def visual_follower(self):
        self.robotHandlerCommandPub.publish("run_visual_follower")

    def open_browser(self):
        self.robotHandlerCommandPub.publish("open_browser")
        url = 'http://192.168.0.100:8080'
        webbrowser.open(url)
    #Create the thread for signal checking and power level
    def start_thread(self):
        number = 0
        self.thread = QThread()
        self.worker = Worker(self._widget, self.robotHandlerCommandPub, number, any, any)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run_thread)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        self.thread.start()


    def skeleton_exit_window(self):

        self.robotHandlerCommandPub.publish('close_skeleton_window')



    def open_new_window_skeleton_detection(self):

        def check_progress():
            bar.setVisible(True)
            label_off.setVisible(False)
            label_on.setVisible(False)
            record_button.setVisible(False)
            is_on = False
            def is_topic_available(topic_name):
                topic_types = rospy.get_published_topics()
                available_topics = [topic[0] for topic in topic_types]
                return topic_name in available_topics
            counter = 0
            if is_topic_available("/body/body_display"):
                bar.setValue(counter + 14)
                counter += 14
                if is_topic_available("/body/mask"):
                    bar.setValue(counter + 14)
                    counter += 14
                    if is_topic_available("/body_posture"):
                        bar.setValue(counter + 14)
                        counter += 14
                        if is_topic_available("/bodydata_process/cmd_vel"):
                            bar.setValue(counter + 14)
                            counter += 14
                            if is_topic_available("/repub/body/body_display/compressed"):
                                bar.setValue(counter + 14)
                                counter += 14
                                bar.setVisible(False)
                                label_on.setVisible(True)
                                is_on = True
                                record_button.setVisible(True)
                            else:
                                counter = 0
                                label_off.setVisible(True)
                                is_on = False
                                record_button.setVisible(False)
                        else:
                            counter = 0
                            label_off.setVisible(True)
                            is_on = False
                            record_button.setVisible(False)

                    else:
                        counter = 0
                        label_off.setVisible(True)
                        is_on = False
                        record_button.setVisible(False)
                else:
                    counter = 0
                    label_off.setVisible(True)
                    is_on = False
                    record_button.setVisible(False)
                    
            else:
                counter = 0
                label_off.setVisible(True)
                is_on = False
                record_button.setVisible(False) 

        def image_callback(msg):
            try:
                cv_image = new_window_1.bridge.compressed_imgmsg_to_cv2(msg)
                height, width, _ = cv_image.shape
                bytes_per_line = 3 * width
                q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
                pixmap = QPixmap.fromImage(q_image)
                new_window_1.videoLabel_1.setPixmap(pixmap)
            except:
                print("Image Callback Error")

        def start_record_thread():
            path1 = "/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/camera_recordings/skeleton_detection/VIDEO-GENERATED_repub_body_body_display_compressed.mp4"
            try:
                if os.path.exists(path1):
                    print("OVERWRITING...")
                    os.remove(path1)
            except:
                print("EXX")

            bag = rosbag.Bag('/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/camera_recordings/skeleton_detection/VIDEO-GENERATED.bag', 'w')
            value = new_window_1.findChild(QSpinBox, 'record_duration').value()

            recording_duration = rospy.Duration.from_sec(value)
            start_time = rospy.Time.now()
            print("Start Time: " + str(start_time))

            def callback(data):
                if rospy.Time.now() - start_time >= recording_duration:
                    # Close the rosbag file when the recording duration ends
                    bag.close()
                    record_label.setVisible(False)
                    record_label_1.setVisible(False)
                    subscriber.unregister()
                    command_1 = ['python2.7', '/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/src/wheeltec-building-inspection/rosbag2video.py', '/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/camera_recordings/skeleton_detection/VIDEO-GENERATED.bag']
                    subprocess.call(command_1) 
                else:
                    bag.write('/repub/body/body_display/compressed', data)

            subscriber = rospy.Subscriber('/repub/body/body_display/compressed', CompressedImage, callback)
        def toggle_record():
            # if not self.thread_1.isRunning():

            # self.worker_1 = Worker(self._widget, self.robotHandlerCommandPub, value, record_label, record_label_1)
            # self.worker_1.moveToThread(self.thread_1)
            # self.thread_1.started.connect(self.worker_1.run_record)
            # self.thread_1.start()
            # self.thread_1.finished.connect(self.thread_1.deleteLater)
            # self.worker_1.finished.connect(self.worker_1.deleteLater)
            thread1 = threading.Thread(target=start_record_thread, args=())
            thread1.start()

            record_label.setVisible(True)
            record_label_1.setVisible(True)
            # else:
            #     print("Recording already in progress...")
            #     print(value)
 
        is_on = False
        self.robotHandlerCommandPub.publish('skel_tracking') 
        new_window_1 = QDialog()
        loadUi("/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/resource/skeleton-tracking-display.ui", new_window_1)
        new_window_1.bridge = CvBridge()
        self.subscriber = rospy.Subscriber('/repub/body/body_display/compressed', CompressedImage, image_callback)
        #self.subscriber_1 = rospy.Subscriber('/repub/body/body_display/compressed', CompressedImage, record_image_callback)
        label_on = new_window_1.findChild(QLabel, 'skeleton_on_label')
        label_off = new_window_1.findChild(QLabel, 'skeleton_off_label')
        bar = new_window_1.findChild(QProgressBar, 'skeleton_prog_bar')
        record_button = new_window_1.findChild(QPushButton, 'record_skeleton')
        record_label = new_window_1.findChild(QLabel, 'record_label')
        record_label_1 = new_window_1.findChild(QLabel, 'record_label_1')
        record_label.setVisible(False)
        record_label_1.setVisible(False)
        record_button.clicked.connect(toggle_record)
        new_window_1.timer = QTimer()
        new_window_1.timer.start(30)

        new_window_1.findChild(QPushButton, 'check_status').clicked.connect(check_progress)
        check_progress()
        
        #new_window_1.finished.connect(self.skeleton_exit_window)

        new_window_1.exec_()
    def obj_detection_exit_window(self):
        self.robotHandlerCommandPub.publish('close_obj_window')

    def open_new_window_object_detection(self):

        object_list = []

        def image_callback(msg):
            if isinstance(msg, Image):

                try:
                        cv_image = new_window.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                        #cv_image = new_window.bridge.imgmsg_to_cv2(msg)
                        height, width, _ = cv_image.shape
                        bytes_per_line = 3 * width
                        qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
                        pixmap = QPixmap.fromImage(qt_image)
                        new_window.videoLabel.setPixmap(pixmap)
                except CvBridgeError as e:
                    print("Error converting Image message:", e)
                except Exception as e:
                    print("Unexpected error:", e)
                
            else:
                print("Msg Type Error")

        def updateObjectDetectedList(msg):
            try:
                match = re.search(r'Class:\s*"([^"]+)"', str(msg))
                found_object = match.group(1)
                if found_object != "" or found_object != " ":
                    object_list.append(found_object)
                    list1 = list(dict.fromkeys(object_list))
                    list_wid.setRowCount(len(list1))
                    list_wid.setVerticalHeaderLabels(list1)
            except:
                print("Object detection list error")

        def checkProgress():
            bar.setVisible(True)
            label_off.setVisible(False)
            label_on.setVisible(False)
            def is_topic_available(topic_name):
                topic_types = rospy.get_published_topics()
                available_topics = [topic[0] for topic in topic_types]
                return topic_name in available_topics
            counter = 0
            if is_topic_available("/darknet_ros/bounding_boxes"):
                bar.setValue(counter + 16)
                counter += 16
                if is_topic_available("/darknet_ros/check_for_objects/feedback"):
                    bar.setValue(counter + 16)
                    counter += 16
                    if is_topic_available("/darknet_ros/check_for_objects/result"):
                        bar.setValue(counter + 16)
                        counter += 16
                        if is_topic_available("/darknet_ros/check_for_objects/status"):
                            bar.setValue(counter + 16)
                            counter += 16
                            if is_topic_available("/darknet_ros/detection_image"):
                                bar.setValue(counter + 16)
                                counter += 16
                                if is_topic_available("/darknet_ros/found_object"):
                                    bar.setValue(counter + 20)
                                    counter += 20
                                    bar.setVisible(False)
                                    label_on.setVisible(True)

                                else:
                                    counter = 0
                                    label_off.setVisible(True)

                            else:
                                counter = 0
                                label_off.setVisible(True)
                        else:
                            counter = 0
                            label_off.setVisible(True)

                    else:
                        counter = 0
                        label_off.setVisible(True)
                else:
                    counter = 0
                    label_off.setVisible(True)
                    
            else:
                counter = 0
                label_off.setVisible(True)
        
        self.robotHandlerCommandPub.publish('obj_detection') 
        new_window = QDialog()
        loadUi("/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/resource/object-detection-display.ui", new_window)
        
        list_wid = new_window.findChild(QTableWidget, 'object_found_list')
        list_wid.setColumnCount(1)
        list_wid.setHorizontalHeaderLabels(['Number'])
        label_off = new_window.findChild(QLabel, 'darknet_off_label')

        self.objectDetectedListSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, updateObjectDetectedList)
        bar = new_window.findChild(QProgressBar, 'darknet_node_prog_bar')
        label_on = new_window.findChild(QLabel, 'darknet_on_label')

        new_window.bridge = CvBridge()
        
        self.subscriber = rospy.Subscriber('/darknet_ros/detection_image', Image, image_callback)
        
        new_window.timer = QTimer()        
        new_window.timer.start(30)




        new_window.findChild(QPushButton, 'check_topics').clicked.connect(checkProgress)
        
        checkProgress()
        #new_window.finished.connect(self.obj_detection_exit_window)
        new_window.exec_()

    
    def load_coordinates(self):
        saved_coords = {}
        with open(COORDINATE_FILE, "rb") as f:
            saved_coords = pickle.load(f)
        return saved_coords


    def save_coordinates(self, coords, name=rospy.get_time()):
        coords.append(0)
        coords.append(0)
        old_data = self.load_coordinates()
        old_data.update({name: tuple([round(coord, 3) for coord in coords])})
        with open(COORDINATE_FILE, "wb") as f:
            pickle.dump(old_data, f, protocol=2)

    def open_path_planner(self):
        self.possible_targets = self.load_coordinates()
        self.goals = []

        path_node_input = self.path_plan_ui.findChild(QComboBox, 'path_node_input')
        path_node_input.clear()
        path_node_input.addItems(self.possible_targets.keys())
        path_node_input.addItem("Custom")

        custom_x_input = self.path_plan_ui.findChild(QDoubleSpinBox, 'custom_x_input')
        custom_y_input = self.path_plan_ui.findChild(QDoubleSpinBox, 'custom_y_input')
        go_button = self.path_plan_ui.findChild(QPushButton, 'go_button')
        reset_button = self.path_plan_ui.findChild(QPushButton, 'reset_button')
        submit_button = self.path_plan_ui.findChild(QPushButton, 'submit_button')
        image_collection_input = self.path_plan_ui.findChild(QComboBox, 'image_collection_input')
        orientation_input = self.path_plan_ui.findChild(QSpinBox, 'orientation_input')
        wait_time_input = self.path_plan_ui.findChild(QSpinBox, 'wait_time_input')

        goal_manager = self.path_plan_ui.findChild(QListWidget, 'goal_list')
        goal_manager.clear()

        def manage_input(data):
            if data == "Custom":
                custom_x_input.setEnabled(True)
                custom_y_input.setEnabled(True)
            else:
                custom_x_input.setEnabled(False)
                custom_y_input.setEnabled(False)

        def reset_targets_widget(hard = False):
            if hard:
                self.goals = []
                goal_manager.clear()
            custom_x_input.setValue(0)
            custom_y_input.setValue(0)
            orientation_input.setValue(0)
            wait_time_input.setValue(0)


        def navigate_to_targets():
            # Use shallow copy to avoid overwriting the goals
            self.task_planning_go([goal.return_data() for goal in self.goals])
            self.goals = []

        def append_new_target():
            try:
                coords = self.possible_targets[path_node_input.currentText()]
            except KeyError:
                coords = custom_x_input.value(), custom_y_input.value(), 0, 1
            point = Point_input(coords, orientation_input.value(), image_collection_input.currentText(), wait_time_input.value())
            self.goals.append(point)
            goal_manager.addItems([str(point)])
            reset_targets_widget()

        submit_button.pressed.connect(
            append_new_target
        )

        path_node_input.currentTextChanged.connect(
            manage_input
        )

        reset_button.pressed.connect(
            lambda: reset_targets_widget(True)
        )

        go_button.pressed.connect(
            navigate_to_targets
        )

        self.path_planner_window.exec_()


    def handle_commands(self, data):
        cmd = data.data
        if cmd == "halt 1":
            self.reset_velocities()

    def update_velocity(self):
        msg = Twist()
        msg.linear.x = float(self._widget.findChild(QSlider, 'linear_velocity_slider').value() / 100.0)
        msg.linear.y = 0
        msg.linear.z = 0

        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = float(self._widget.findChild(QSlider, 'angular_velocity_slider').value() / 100.0)

        self.velocityCommandPub.publish(msg)

        self._widget.findChild(QLabel, 'current_linear_velocity').setText(str(msg.linear.x) + " m/s")
        self._widget.findChild(QLabel, 'current_angular_velocity').setText(str(msg.angular.z) + " m/s")

    def update_velocity_slider(self, velocity, type):
        slider = self._widget.findChild(QSlider, velocity + "_velocity_slider")
        if type == "+":
            slider.setValue(slider.value() + 10)
        elif type == "-":
            slider.setValue(slider.value() - 10)

    def reset_velocities(self):
        zero_vector = Twist()
        zero_vector.linear.x = 0
        zero_vector.linear.y = 0
        zero_vector.linear.z = 0
        zero_vector.angular.x = 0
        zero_vector.angular.y = 0
        zero_vector.angular.z = 0

        self.velocityCommandPub.publish(zero_vector)

        self._widget.findChild(QSlider, 'linear_velocity_slider').setValue(0)
        self._widget.findChild(QSlider, 'angular_velocity_slider').setValue(0)

    def set_max_slider(self, velocity):
        input_box = self._widget.findChild(QDoubleSpinBox, 'max_' + velocity + '_speed_input')
        self._widget.findChild(QSlider, velocity + '_velocity_slider').setMaximum(input_box.value() * 100)
        self._widget.findChild(QSlider, velocity + '_velocity_slider').setMinimum(-input_box.value() * 100)
        input_box.clearFocus()
    
    def update(self):
        if not self.proc_manager.is_subprocess_running('data_transfer') and \
                self.transfer_state == TransferState.TRANSFER:
            self.add_to_log_console(String('Data cap transfer complete'))
            self.transfer_state = TransferState.IDLE

    def refresh_robot_cap_list(self, cap_files_message):
        cap_file_list = cap_files_message.data.split("|")
        self.fileCapView.clear()
        self.fileCapView.insertItems(0, cap_file_list)

    def transfer_cap_file(self):
        if self.fileCapView.currentItem() is not None:
            # INFO: running SCP requires ssh keys for authenticating access to robot
            if self.proc_manager.create_new_subprocess('data_transfer', 'scp wheeltec@wheeltec:' +
                                                                        ROBOT_CAP_SAVE_DIRECTORY +
                                                                        self.device_selection.currentText() + '/' +
                                                                        self.fileCapView.currentItem().text() + " " +
                                                                        LAB_PC_CAP_TRANSFER_DIRECTORY +
                                                                        self.device_selection.currentText()):
                self.add_to_log_console(String('Start transfer of data cap'))
                self.transfer_state = TransferState.TRANSFER
            else:
                self.add_to_log_console(String('Data cap already in transmission'))

    def add_to_log_console(self, log_message):
        self.log_console.addItems([log_message.data])
        self.log_console.scrollToBottom()

################################################################################
    #Enable Keyboard
    def keyboard_on(self):        
        self._widget.findChild(QLineEdit, 'InputCmd').textChanged.connect(self.controller_robot)
    #Controller of the Robot using W, A, S, D, Q, E and P
    def controller_robot(self, text):

        linear_velocity = self._widget.findChild(QDoubleSpinBox, 'max_linear_speed_input')
        angular_velocity = self._widget.findChild(QDoubleSpinBox, 'max_angular_speed_input')

        key = text.strip().lower()
        if text.strip():
            if key in ('w', 'a', 's', 'd', 'e', 'q'):
                print("Valid character entered:", key)
                msg = Twist()
                msg.linear.x = 0
                msg.linear.y = 0
                msg.linear.z = 0

                msg.angular.x = 0
                msg.angular.y = 0
                msg.angular.z = 0

                if key == 'w':
                    msg.linear.x += linear_velocity.value()
                elif key == 'a':
                    msg.angular.z += angular_velocity.value()
                elif key == 's':
                    msg.linear.x += -linear_velocity.value()
                elif key == 'd':
                    msg.angular.z += -angular_velocity.value()
                elif key == 'e':
                    msg.linear.y = -0.5
                elif key == 'q':
                    msg.linear.y = 0.5
                    
                self.velocityCommandPub.publish(msg)

            elif key == 'p':
                print("Stop command received.")
                msg = Twist()
                msg.linear.x = 0
                msg.linear.y = 0
                msg.linear.z = 0

                msg.angular.x = 0
                msg.angular.y = 0
                msg.angular.z = 0
                
                self.velocityCommandPub.publish(msg)
        else:
            print("Invalid character entered:", key)

        self._widget.findChild(QLineEdit, 'InputCmd').clear()

    #publishing all the points that I want on the map at the same time.
    def set_all_points(self):
        def is_topic_available(topic_name):
            topic_types = rospy.get_published_topics()
            available_topics = [topic[0] for topic in topic_types]
            return topic_name in available_topics
        
        l1 = self._widget.findChild(QSpinBox, 'spinboxP1').value()
        l2 = self._widget.findChild(QSpinBox, 'spinboxP2').value()
        l3 = self._widget.findChild(QSpinBox, 'spinboxP3').value()
        l4 = self._widget.findChild(QSpinBox, 'spinboxP4').value()

        list_l = [l1,l2,l3,l4]
        

        gps1 = {'x': 0.831, 'y': 0.844, 'z': 0}
        gps2 = {'x': 4.668, 'y': 0.917, 'z': 0}
        gps3 = {'x': 4.705, 'y': -0.257, 'z': 0}
        gps4 = {'x': 0.675, 'y': -0.150, 'z': 0}

        gps = [gps1,gps2,gps3,gps4]

        p1 = gps[list_l[0] - 1]
        p2 = gps[list_l[1] - 1]
        p3 = gps[list_l[2] - 1]
        p4 = gps[list_l[3] - 1]
                
        if is_topic_available("/clicked_point"):
            locations = [
                p1,
                p2,
                p3,
                p4,


                 #{'x': 2.5380358696, 'y': 0.392, 'z': 0}
                # {'x': -13.092622757, 'y': 3.63729691505, 'z': 0.00315856933594},
                # {'x': -12.4344749451, 'y': 18.3455543518, 'z': 0.00156402587891},
                # {'x': -0.678248405457, 'y': 17.7169075012, 'z': 0.00148773193359},
                # {'x': -1.13597512245, 'y': 3.19778370857, 'z': -0.00111389160156},
                # {'x': -1.00027537346, 'y': 17.4356002808, 'z': 0.00460243225098}
            ]
            
            for location in locations:
                msg = PointStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'map'
                msg.point.x = location['x']
                msg.point.y = location['y']
                msg.point.z = location['z']
                
                self.pointClickedCommandPub.publish(msg)

                rospy.sleep(1)
        else:
            print("Can't set points...")

################################################################################

class Worker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)

    def __init__(self, widget, cmd_pub, seconds, label, label1):
        super(Worker, self).__init__()
        self._widget = widget
        self.command = cmd_pub
        self.time = seconds
        self.la = label
        self.la1 = label1

    def run_record(self):
        path1 = "/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/camera_recordings/skeleton_detection/VIDEO-GENERATED_repub_body_body_display_compressed.mp4"
        try:
            if os.path.exists(path1):
                print("OVERWRITING...")
                os.remove(path1)
        except:
            print("EXX")

        bag = rosbag.Bag('/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/camera_recordings/skeleton_detection/VIDEO-GENERATED.bag', 'w')
        recording_duration = rospy.Duration.from_sec(self.time)
        start_time = rospy.Time.now()
        print("Start Time: " + str(start_time))

        def callback(data):
            if rospy.Time.now() - start_time >= recording_duration:
                # Close the rosbag file when the recording duration ends
                bag.close()
                self.la.setVisible(False)
                self.la1.setVisible(False)
                subscriber.unregister()
                command_1 = ['python2.7', '/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/src/wheeltec-building-inspection/rosbag2video.py', '/home/concordia/catkin_ws/src/wheeltec-building-inspection/data_recordings/camera_recordings/skeleton_detection/VIDEO-GENERATED.bag']
                subprocess.call(command_1) 
            else:
                bag.write('/repub/body/body_display/compressed', data)

        subscriber = rospy.Subscriber('/repub/body/body_display/compressed', CompressedImage, callback)
        # rospy.spin()



    def run_thread(self):
        wifi_lcd_strength = self._widget.findChild(QLCDNumber, 'WifiStrength')
        wifi_lcd_strength_2 = self._widget.findChild(QLCDNumber, 'WifiStrength_2')

        entry_x = self._widget.findChild(QLineEdit, 'coordinate_x_output')
        entry_y = self._widget.findChild(QLineEdit, 'coordinate_y_output')
        def set_entry_gps(data):
            x = str(data.x)
            y = str(data.y)
            entry_x.setText(x)
            entry_y.setText(y)
            

        def get_power_level(data):
            power_level_lcd = self._widget.findChild(QLCDNumber, 'power_level')
            voltage = data.data
            power_level_lcd.display(voltage)
                


        def get_wifi_interface():
            result = subprocess.Popen(['iwconfig'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            # Get the output from the command
            output, error = result.communicate()

            # Split the output into lines
            lines = output.split('\n')

            # Find lines containing Wi-Fi interface names
            wifi_interfaces = []
            for line in lines:
                if 'IEEE 802.11' in line:
                    interface = line.split(' ')[0]
                    wifi_interfaces.append(interface)

            # Return the Wi-Fi interface names as a list
            return wifi_interfaces


        def get_wifi_signal_strength(interface):
            try:
                output = subprocess.check_output(['iwconfig', interface])
                output = output.decode('utf-8')
                signal_level = re.search('Signal level=(\d+/\d+|\-\d+)', output)
                if signal_level:
                    return signal_level.group(1)
                else:
                    return None
            except subprocess.CalledProcessError:
                return None
        
        def stopping():
            self.command.publish('stop_all')
   
        # Specify your Wi-Fi interface name

        
        wifi_interface = get_wifi_interface()
        wifi_interface.remove("wlp1s0")
        
        #wifi_interface = 'wlx642943a66a33'
        while True:
            time.sleep(1)
            #print(wifi_interface)
            rospy.Subscriber('PowerVoltage', Float32, get_power_level)
            rospy.Subscriber('tf_echo_translation', Vector3, set_entry_gps)
            try:
                if wifi_interface[0] == "wlx642943a66a33" and wifi_interface[1] == "wlx0c9d92b71f61":
                    signal_strength = get_wifi_signal_strength(wifi_interface[0])
                    signal_strength_2 = get_wifi_signal_strength(wifi_interface[1])
                elif wifi_interface[1] == "wlx642943a66a33" and wifi_interface[0] == "wlx0c9d92b71f61":
                    signal_strength = get_wifi_signal_strength(wifi_interface[1])
                    signal_strength_2 = get_wifi_signal_strength(wifi_interface[0])
                else:
                    signal_strength = get_wifi_signal_strength(wifi_interface[0])
                    signal_strength_2 = get_wifi_signal_strength(wifi_interface[0])
                    #print("EXCEPTION!")
            except:
                signal_strength = get_wifi_signal_strength(wifi_interface[0])
                signal_strength_2 = get_wifi_signal_strength(wifi_interface[0])
                #print("EXCCCC")

            # signal_strength = get_wifi_signal_strength(wifi_interface[0])
            # signal_strength_2 = get_wifi_signal_strength(wifi_interface[0])


            if signal_strength is not None:
                if "/" in signal_strength:
                    numerator, denominator = signal_strength.split("/")
                    signal_strength = int(numerator) * 100 // int(denominator)  # Convert to percentage
                else:
                    signal_strength = int(signal_strength)
                    # print(signal_strength)
                    # print(signal_strength_2)
                if signal_strength <= 100 and signal_strength >= 60:
                    wifi_lcd_strength.setStyleSheet("QLCDNumber { background-color: black; color: lime }")
                elif signal_strength < 60 and signal_strength >= 40:
                    wifi_lcd_strength.setStyleSheet("QLCDNumber { background-color: black; color: yellow }")
                elif signal_strength < 40 and signal_strength >= 25 :
                    wifi_lcd_strength.setStyleSheet("QLCDNumber { background-color: black; color: red }")   
                else:
                    #print("Stopping all operations becasuse signal is weak...")
                    wifi_lcd_strength.setStyleSheet("QLCDNumber { background-color: white; color: aqua }")
                    #stopping()
                    #break

                wifi_lcd_strength.display(signal_strength)
                wifi_lcd_strength_2.display(signal_strength_2)
                # print("Signal strength: " + str(signal_strength) + "%")
                
            else:
                pass
                # print('Unable to retrieve signal strength.')

