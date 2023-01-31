#!/usr/bin/env python

import sys

from rqt_mypkg.my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'building-inspec'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
