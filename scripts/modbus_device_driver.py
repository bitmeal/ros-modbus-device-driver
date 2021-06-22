#!/usr/bin/env python

from __future__ import print_function

import logging

import rospy

from ros_modbus_device import *


logger = logging.getLogger(__name__)
ROSLoggingAdapter.attach(__name__)

ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
logger.addHandler(ch)

rospy.init_node('modbus_device_driver', anonymous=True)

modbus_device = ROSModbusSlaveDevice()

modbus_device.connect()
modbus_device.run()
