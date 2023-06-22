import os
import rospy
import rospkg

import subprocess
import re
import time

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import CompressedImage

from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QDialog

from PyQt5.QtWidgets import QPushButton, QListWidget, QComboBox, QDoubleSpinBox, QShortcut, QRadioButton, QSlider, QLabel, QCheckBox, QLineEdit, QLCDNumber, QTableWidget, QProgressBar
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, QObject
from PyQt5.QtGui import QColor, QImage, QPixmap

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from utils.Definitions import *
from utils.SubProcessManager import *

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
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('wheeltec-building-inspection-ui'), 'resource',
                               'building-inspec-control.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('NavControlUi')

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
        
        self._widget.findChild(QRadioButton, 'EnableCamButton').toggled.connect(
            lambda: self.robotHandlerCommandPub.publish('start_cam')
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

    #Create the thread for signal checking and power level
    def start_thread(self):
        self.thread = QThread()
        self.worker = Worker(self._widget, self.robotHandlerCommandPub)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run_thread)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        self.thread.start()

    def open_new_window_skeleton_detection(self):
        def check_progress():
            bar.setVisible(True)
            label_off.setVisible(False)
            label_on.setVisible(False)
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


        def image_callback(msg):
            cv_image = new_window_1.bridge.compressed_imgmsg_to_cv2(msg)
            height, width, _ = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_image)
            new_window_1.videoLabel_1.setPixmap(pixmap)
        
        self.robotHandlerCommandPub.publish('skel_tracking') 
        new_window_1 = QDialog()
        loadUi("/home/concordia/catkin_ws/src/wheeltec-building-inspection/UI/resource/skeleton-tracking-display.ui", new_window_1)
        new_window_1.bridge = CvBridge()
        self.subscriber = rospy.Subscriber('/repub/body/body_display/compressed', CompressedImage, image_callback)
        label_on = new_window_1.findChild(QLabel, 'skeleton_on_label')
        label_off = new_window_1.findChild(QLabel, 'skeleton_off_label')
        bar = new_window_1.findChild(QProgressBar, 'skeleton_prog_bar')
        new_window_1.timer = QTimer()
        new_window_1.timer.start(30)

        new_window_1.findChild(QPushButton, 'check_status').clicked.connect(check_progress)
        check_progress()

        new_window_1.exec_()

    def open_new_window_object_detection(self):

        object_list = []

        def image_callback(msg):
            cv_image = new_window.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape
            bytes_per_line = 3 * width
            qt_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            new_window.videoLabel.setPixmap(QPixmap.fromImage(qt_image))
        def updateObjectDetectedList(msg):
            match = re.search(r'Class:\s*"([^"]+)"', str(msg))
            found_object = match.group(1)
            object_list.append(found_object)
            list1 = list(dict.fromkeys(object_list))
            list_wid.setRowCount(len(list1))
            list_wid.setVerticalHeaderLabels(list1)

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
        self.objectDetectedListSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, updateObjectDetectedList)
        bar = new_window.findChild(QProgressBar, 'darknet_node_prog_bar')
        label_on = new_window.findChild(QLabel, 'darknet_on_label')
        label_off = new_window.findChild(QLabel, 'darknet_off_label')

        new_window.bridge = CvBridge()
        self.subscriber = rospy.Subscriber('/darknet_ros/detection_image', Image, image_callback)
        new_window.timer = QTimer()
        new_window.timer.start(30)

        new_window.findChild(QPushButton, 'check_topics').clicked.connect(checkProgress)
        checkProgress()
        new_window.exec_()


   
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

################################################################################
    #Enable Keyboard
    def keyboard_on(self):        
        self._widget.findChild(QLineEdit, 'InputCmd').textChanged.connect(self.controller_robot)
    #Controller of the Robot using W, A, S, D and P
    def controller_robot(self, text):
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
                    msg.linear.x += 0.5
                elif key == 'a':
                    msg.angular.z += 0.5
                elif key == 's':
                    msg.linear.x += -0.5
                elif key == 'd':
                    msg.angular.z += -0.5
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
        if is_topic_available("/clicked_point"):
            locations = [
                {'x': -13.092622757, 'y': 3.63729691505, 'z': 0.00315856933594},
                {'x': -12.4344749451, 'y': 18.3455543518, 'z': 0.00156402587891},
                {'x': -0.678248405457, 'y': 17.7169075012, 'z': 0.00148773193359},
                {'x': -1.13597512245, 'y': 3.19778370857, 'z': -0.00111389160156},
                {'x': -1.00027537346, 'y': 17.4356002808, 'z': 0.00460243225098}
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
#Signal Checker Class
class Worker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)

    def __init__(self, widget, cmd_pub):
        super(Worker, self).__init__()
        self._widget = widget
        self.command = cmd_pub
    
    def run_thread(self):
        wifi_lcd_strength = self._widget.findChild(QLCDNumber, 'WifiStrength')

        def get_power_level(data):
            power_level_lcd = self._widget.findChild(QLCDNumber, 'power_level')
            voltage = data.data
            power_level_lcd.display(voltage)
                

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
        wifi_interface = 'wlx642943a66a33' 
        while True:
            time.sleep(1)
            rospy.Subscriber('PowerVoltage', Float32, get_power_level)

            signal_strength = get_wifi_signal_strength(wifi_interface)
            if signal_strength is not None:
                if "/" in signal_strength:
                    numerator, denominator = signal_strength.split("/")
                    signal_strength = int(numerator) * 100 // int(denominator)  # Convert to percentage
                else:
                    signal_strength = int(signal_strength)
                    print(signal_strength)
                if signal_strength <= 100 and signal_strength >= 60:
                    wifi_lcd_strength.setStyleSheet("QLCDNumber { background-color: black; color: lime }")
                elif signal_strength < 60 and signal_strength >= 40:
                    wifi_lcd_strength.setStyleSheet("QLCDNumber { background-color: black; color: yellow }")
                elif signal_strength < 40 and signal_strength >= 25 :
                    wifi_lcd_strength.setStyleSheet("QLCDNumber { background-color: black; color: red }")   
                else:
                    print("Stopping all operations becasuse signal is weak...")
                    wifi_lcd_strength.setStyleSheet("QLCDNumber { background-color: white; color: aqua }")
                    #stopping()
                    #break

                wifi_lcd_strength.display(signal_strength)
                print("Signal strength: " + str(signal_strength) + "%")
                
            else:
                print('Unable to retrieve signal strength.')

