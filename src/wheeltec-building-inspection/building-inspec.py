import os
import rospy
import rospkg

from std_msgs.msg import String

from PyQt5.QtWidgets import QPushButton

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

        # Likely don't need
         # # Process standalone plugin command-line arguments
         # from argparse import ArgumentParser
         # parser = ArgumentParser()
         # # Add argument(s) to the parser.
         # parser.add_argument("-q", "--quiet", action="store_true",
         #               dest="quiet",
         #               help="Put plugin in silent mode")
         # args, unknowns = parser.parse_known_args(context.argv())
         # if not args.quiet:
         #     print 'arguments: ', args
         #     print 'unknowns: ', unknowns
 
         # Create QWidget
         self._widget = QWidget()
         # Get path to UI file which should be in the "resource" folder of this package
         ui_file = os.path.join(rospkg.RosPack().get_path('wheeltec-building-inspection'), 'resource', 'building-inspec-control.ui')
         # Extend the widget with all attributes and children from UI file
         loadUi(ui_file, self._widget)
         # Give QObjects reasonable names
         self._widget.setObjectName('NavControlUi')
         # Show _widget.windowTitle on left-top of each plugin (when 
         # it's set in _widget). This is useful when you open multiple 
         # plugins at once. Also if you open multiple instances of your 
         # plugin at once, these lines add number to make it easy to 
         # tell from pane to pane.
         if context.serial_number() > 1:
             self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
         # Add widget to the user interface
         context.add_widget(self._widget)

         # Event handlers for buttons
         self._widget.findChild(QPushButton, 'StartMapping').clicked.connect(lambda: self.robotHandlerCommandPub.publish('start_mapping'))
         self._widget.findChild(QPushButton, 'SaveMap').clicked.connect(lambda: self.robotHandlerCommandPub.publish('save_map'))
         self._widget.findChild(QPushButton, 'StartNav').clicked.connect(lambda: self.robotHandlerCommandPub.publish('start_nav'))
         self._widget.findChild(QPushButton, 'StopAll').clicked.connect(lambda: self.robotHandlerCommandPub.publish('stop_all'))
 
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


