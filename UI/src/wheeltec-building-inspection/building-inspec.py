import os
import rospy
import rospkg

from std_msgs.msg import String

from PyQt5.QtWidgets import QPushButton, QListWidget, QComboBox, QDoubleSpinBox, QShortcut
from PyQt5.QtCore import QTimer

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

        # Register subscribers
        self.robotHandlerStatusSub = rospy.Subscriber('robot_handler_status', String, self.add_to_log_console)
        self.robotHandlerCapListRequest = rospy.Subscriber('robot_handler_cap_file_list', String,
                                                           self.refresh_robot_cap_list)

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

        self._widget.findChild(QPushButton, 'DelCapButton').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('delete_data_cap ' + self.device_selection.currentText() + ' '
                                                        + (self.fileCapView.currentItem().text() if
                                                           self.fileCapView.currentItem() is not None else 'None')))

        self._widget.findChild(QPushButton, 'TransferCapButton').clicked.connect(
            lambda: self.transfer_cap_file())

        # General
        self._widget.findChild(QPushButton, 'StopAll').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('stop_all'))

        self._widget.findChild(QPushButton, 'Halt').clicked.connect(
            lambda: self.robotHandlerCommandPub.publish('halt'))
        self.halt_shortcut = QShortcut('k', self._widget)
        self.halt_shortcut.activated.connect(
            lambda: self.robotHandlerCommandPub.publish('halt'))

        self._widget.findChild(QDoubleSpinBox, 'maxSpeedInput').editingFinished.connect(
            self.set_max_speed
        )

        # Handle log console
        self.log_console = self._widget.findChild(QListWidget, 'LogConsole')

        # TODO: Add update method for checking status of subprocesses using QTimer object
        self.timer = QTimer(500)
        self.timer.timeout.connect(self.update)
        self.timer.start()
    
    def set_max_speed(self):
        input_box = self._widget.findChild(QDoubleSpinBox, 'maxSpeedInput')
        self.robotHandlerCommandPub.publish('set_max_speed ' + str(input_box.value()))
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
