from functools import partial

import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Int64
from std_msgs.msg import UInt8
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import UInt64
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Char
from std_msgs.msg import String
from ros_modbus_device_driver.msg import Byte as BYTE
from ros_modbus_device_driver.msg import Word as WORD
from ros_modbus_device_driver.msg import Lword as LWORD
from ros_modbus_device_driver.msg import Dword as DWORD

class ROSCommunicationManager:
    mapping = {
        '#DISCRETE': Bool,
        'BYTE'  : BYTE,
        'WORD'	: WORD,
        'DWORD'	: DWORD,
        'LWORD'	: LWORD,
        'SINT'	: Int8,
        'INT'	: Int16,
        'DINT'	: Int32,
        'LINT'	: Int64,
        'USINT'	: Int8,
        'UINT'	: Int16,
        'UDINT'	: Int32,
        'ULINT'	: Int64,
        'REAL'	: Float32,
        'LREAL'	: Float64,
        'CHAR'	: Char,
        'STRING': String
    }

    def __init__(self, ros_modbus_slave_device):
        self.publishers = []
        self.subscribers = []

        self.device = ros_modbus_slave_device
    
    # all make_attach_* methods get passed an input object and have to
    # return a callable object with one parameter (taking the input value)
    def make_attach_publisher(self, input):
        pub = rospy.Publisher(self.device.dev_name(input.name), self.mapping[input.type], queue_size=10)
        self.publishers.append(pub)
        return pub.publish
        
    def make_link_subscriber(self, input, modbus_device):
        self.subscribers.append(rospy.Subscriber(self.device.dev_name(input.name) + '/write', self.mapping[input.type], lambda msg: modbus_device.write(input.name, msg.data)))
    