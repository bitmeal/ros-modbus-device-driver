#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'ros_modbus_device_driver',
        'ros_modbus_device',
        'modbus_device'
    ],
    package_dir={
        'modbus_device': 'src/modbus_device',
        'ros_modbus_device': 'src/ros_modbus_device'
    }
    # scripts=[
    #     'scripts/modbus_device_driver.py',
    # ]
)

setup(**d)
