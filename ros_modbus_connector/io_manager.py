from ros_modbus_connector.util import *
from ros_modbus_connector.mapping_discrete import *
from ros_modbus_connector.mapping_register import *
from ros_modbus_connector.reader_discrete import *
from ros_modbus_connector.reader_register import *

from pymodbus.constants import Endian

import math


class IOManager:
    # init from config
    def __init__(self, client, config):
        self.inputs = {}
        self.outputs = {}

        self.discrete_inputs = []
        self.coils = []
        self.input_registers = []
        self.holding_registers = []

        self.readers = []

        self.client = client
        self.unit = config['unit']
        self.config = config

        # decoding config
        self.byteorder = Endian.Big if not config.get('byteorder_reverse', False) == True else Endian.Little
        self.wordorder = Endian.Little if not config.get('wordorder_reverse', False) == True else Endian.Big

        # read operation generator config
        self.discrete_continuous = config.get('discrete_read_continuous', False)
        self.discrete_continuous_gap = config.get('discrete_read_separation_gap', 64)
        self.registers_continuous = config.get('registers_read_continuous', False)
        self.registers_continuous_gap = config.get('registers_read_separation_gap', 12)

        self.__generate_mappings()
        self.__generate_readers()

        self.inputs = {i.name: i for i in [*self.discrete_inputs, *self.coils, *self.input_registers, *self.holding_registers]}
        self.outputs = {i.name: i for i in [*self.coils, *self.holding_registers]}
    
    def __generate_mappings(self):
        self.discrete_inputs = [
            DiscreteInputMapping(**{"name": name, "address": address})
            for name, address in self.config['mapping'].get('discrete_inputs', {}).items()
        ]
        self.coils = [
           CoilMapping(**{"name": name, "address": address})
            for name, address in self.config['mapping'].get('coils', {}).items()
        ]

        self.input_registers = [
            InputRegisterMapping(**{**{"name": name, "byteorder": self.byteorder, "wordorder": self.wordorder}, **config})
            for name, config in self.config['mapping'].get('input_registers', {}).items()
        ]
        self.holding_registers = [
            HoldingRegisterMapping(**{**{"name": name, "byteorder": self.byteorder, "wordorder": self.wordorder}, **config})
            for name, config in self.config['mapping'].get('holding_registers', {}).items()
        ]

    def __generate_readers(self):
        # sort all mappings in ascending order
        self.discrete_inputs.sort(key=lambda e: e.address)
        self.coils.sort(key=lambda e: e.address)
        # sort register mappings by address + offset:
        # offset == true, indicates a read from the second byte being transmitted for one register;
        # effectively requiring a dummy read/padding from the previous full word read, when no
        # non-offset byte is to be read from the same register
        self.input_registers.sort(key=lambda e: e.address + 0.5*e.offset)
        self.holding_registers.sort(key=lambda e: e.address + 0.5*e.offset)

        # group discrete mappings and registers into single read operations, with gaps
        # no bigger than _continuous_gap. When continuous read is false, gap = 1
        def group_discrete(a, b):
            return b.address - a.address == 1 or (self.discrete_continuous and b.address - a.address < self.discrete_continuous_gap)

        def group_registers(a, b):
            return a.address + math.ceil(a.size) >= b.address or (self.registers_continuous and b.address - (a.address + math.ceil(a.size))  < self.registers_continuous_gap)

        self.readers += [DiscreteRangeReader(group) for group in group_list(group_discrete, self.discrete_inputs)]
        self.readers += [CoilRangeReader(group) for group in group_list(group_discrete, self.coils)]

        endianness = {"byteorder": self.byteorder, "wordorder": self.wordorder}
        self.readers += [InputRegisterRangeReader(group, **endianness) for group in group_list(group_registers, self.input_registers)]
        self.readers += [HoldingRegisterRangeReader(group, **endianness) for group in group_list(group_registers, self.holding_registers)]

    def write(self, mapping, value):
        self.outputs[mapping].write(self.client, value, self.unit)
    
    def read(self):
        for reader in self.readers:
            reader.read(self.client, self.unit)
    
    # calls handler_function with each input object and stores the returned value as a callback
    # the return value of handler_function is thus expected to be a callable. the value of the
    # register/coil will be passed as a single positional parameter when called.
    def attach_callbacks_discrete(self, handler_function):
        for i in [*self.discrete_inputs, *self.coils]:
            i.attach_callback(handler_function(i))

    def attach_callbacks_registers(self, handler_function):
        for i in [*self.input_registers, *self.holding_registers]:
            i.attach_callback(handler_function(i))

    def attach_callbacks(self, handler_function):
        self.attach_callbacks_discrete(handler_function)
        self.attach_callbacks_registers(handler_function)


