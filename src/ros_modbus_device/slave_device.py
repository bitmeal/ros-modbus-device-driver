import logging
logger = logging.getLogger(__name__)
import rospy

from ros_modbus_device.slave_device_config import ROSModbusSlaveDeviceConfig
from ros_modbus_device.ros_communication_manager import ROSCommunicationManager
from modbus_device import ModbusSlaveDevice


# to setup the modbus device as part of your node, you may
# push the device down in your own nodes namespace. if no
# namespace is provided, topics will be published at absolute
# paths, with the namespace taken from the slaves config file
class ROSModbusSlaveDevice:
    def __init__(self, namespace=None):
        if namespace:
            ns = namespace.strip('/')
            # push down, or insert directly when '/' was given
            self.ns = (ns + '/') if len(ns) else ''
        else:
            self.ns = None

        self.config = ROSModbusSlaveDeviceConfig(self)
        self.name = self.config.config['name']
        self.rate = rospy.Rate(self.config.config['rate'])

        self.ros_comm_mgr = ROSCommunicationManager(self)
        self.modbus_slave = ModbusSlaveDevice(self.config.config)
        self.modbus_slave.attach_callbacks(self.ros_comm_mgr.make_attach_publisher)
        for output in self.modbus_slave.outputs.values():
            self.ros_comm_mgr.make_link_subscriber(output, self.modbus_slave)


    # return a name relative to the nodes namespace; including
    # the configured namespace for the modbus device within the node
    def ns_name(self, name):
        return '~{}{}'.format(self.ns if self.ns else '', name.strip('/'))
    
    # return name in node namespace, respecting child namespace for the
    # device, IF configured. IF no namespace for the device is configured
    # the name from the device configuration/parameter/commandline will be used
    def dev_name(self, name):
        return ns_name(name) if self.ns else '/{}/{}'.format(self.name, name.strip('/'))

    def connect(self):
        self.modbus_slave.connect()
    
    def run(self):
        while not rospy.is_shutdown():
            self.modbus_slave.read()
            self.rate.sleep()


    