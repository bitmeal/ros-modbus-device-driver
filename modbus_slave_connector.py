#!/usr/bin/env python3

# from ros_modbus_connector.modbus_slave_device import ModbusSlaveDevice
from ros_modbus_connector import ModbusSlaveDevice

import json
import time

# run standalone
if __name__ == "__main__":
    with open("./slave.json") as config_file:
        config = json.load(config_file)

        slave = ModbusSlaveDevice(config)

        def make_print_callback(input):
            return lambda v: print(input.name, v)
        
        slave.attach_callbacks(make_print_callback)
        slave.connect()

        gripper_state = False

        while True:
            slave.read()
            time.sleep(0.5)
            slave.write("gripper", gripper_state)
            gripper_state = not gripper_state
    
