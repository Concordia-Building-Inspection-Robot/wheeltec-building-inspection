import os
import rospy
import rospkg

from std_msgs.msg import String

from PyQt5.QtWidgets import QPushButton, QListWidget, QComboBox, QFileSystemModel
from PyQt5.QtCore import QTimer

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

class NavControl(Plugin):
     def __init__(self, context):
         super(NavControl, self).__init__(context)
         # Give QObjects reasonable names
         self.setObjectName('NavControl')

         # Register publishers
         self.robotHandlerCommandPub = rospy.Publisher('robot_handler_cmd', String, queue_size=10)

         # Register subscribers
         self.robotHandlerStatusSub = rospy.Subscriber('robot_handler_status', String, self.add_to_log_console)
         self.robotHandlerCapListRequest = rospy.Subscriber('robot_handler_cap_file_list', String, self.refresh_robot_cap_list)
 
         # Create QWidget
         self._widget = QWidget()
         ui_file = os.path.join(rospkg.RosPack().get_path('wheeltec-building-inspection-ui'), 'resource', 'building-inspec-control.ui')
         loadUi(ui_file, self._widget)
         self._widget.setObjectName('NavControlUi')

         if context.serial_number() > 1:
             self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
         # Add widget to the user interface
         context.add_widget(self._widget)

         # TODO: Use loop to create handlers
         # controlCommands = [{'button_name': 'StartManualControl', 'command': 'start_manual_control'},
         #                    {'button_name': 'StartMapping', 'command': 'start_mapping'},
         #                    {'button_name': 'SaveMap', 'command': 'save_map'},
         #                    {'button_name': 'StartNav', 'command': 'start_nav'},
         #                    {'button_name': 'StopAll', 'command':'stop_all'}]

         # Event handlers for buttons
         # NavControl Tab
         # self._widget.findChild(QPushButton, 'StartManualControl').clicked.connect(
         #     lambda: self.robotHandlerCommandPub.publish('start_manual_control'))

         self._widget.findChild(QPushButton, 'StartNormalSlam').clicked.connect(
             lambda: self.robotHandlerCommandPub.publish('start_normal_slam'))

         self._widget.findChild(QPushButton, 'StartMapping').clicked.connect(
             lambda: self.robotHandlerCommandPub.publish('start_mapping'))

         self._widget.findChild(QPushButton, 'SaveMap').clicked.connect(
             lambda: self.robotHandlerCommandPub.publish('save_map'))

         self._widget.findChild(QPushButton, 'StartNav').clicked.connect(
             lambda: self.robotHandlerCommandPub.publish('start_nav'))

         # Data Cap Tab
         self.device_selection = self._widget.findChild(QComboBox, 'selectDeviceBox')
         self.device_selection.addItem('lidar')

         self.fileCapView = self._widget.findChild(QListWidget, 'ListCaptureFiles')
         self.robotHandlerCommandPub.publish('get_cap_file_list ' + self.device_selection.currentText())

         self._widget.findChild(QPushButton, 'ToggleCollectionButton').clicked.connect(
             lambda: self.robotHandlerCommandPub.publish('toggle_collection ' + self.device_selection.currentText()))

         self._widget.findChild(QPushButton, 'TogglePlaybackButton').clicked.connect(
             lambda: self.robotHandlerCommandPub.publish('toggle_playback ' + self.device_selection.currentText() + ' '
                                                         + (self.fileCapView.currentItem().text() if
                                                            self.fileCapView.currentItem() is not None else 'None')))

         # General
         self._widget.findChild(QPushButton, 'StopAll').clicked.connect(
             lambda: self.robotHandlerCommandPub.publish('stop_all'))

         # Handle log console
         self.log_console = self._widget.findChild(QListWidget, 'LogConsole')


     def refresh_robot_cap_list(self, cap_files_message):
        cap_file_list = cap_files_message.data.split("|")
        self.fileCapView.clear()
        self.fileCapView.insertItems(0, cap_file_list)

     def shutdown_plugin(self):
         # TODO unregister all publishers here
         pass
 
     def save_settings(self, plugin_settings, instance_settings):
         # TODO save intrinsic configuration, usually using:
         # instance_settings.set_value(k, v)
         pass

     def restore_settings(self, plugin_settings, instance_settings):
         # TODO restore intrinsic configuration, usually using:
         # v = instance_settings.value(k)
         pass
 
     #def trigger_configuration(self):
         # Comment in to signal that the plugin has a way to configure
         # This will enable a setting button (gear icon) in each dock widget title bar
         # Usually used to open a modal configuration dialog

     def add_to_log_console(self, log_message):
         self.log_console.addItems([log_message.data])

