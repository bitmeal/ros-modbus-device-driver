#!/usr/bin/env python3

import json
import math
import time
import functools

from pymodbus.constants import Endian
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.payload import BinaryPayloadBuilder
from struct import pack


# UTIL
def group_list(predicate, list):
    return functools.reduce( (lambda acc, e:
            [*acc[0:-1], [*acc[-1], e]] if len(acc) and predicate(acc[-1][-1], e) else [*acc, [e]]
        ), list, [] )



######################

class IOManager:
    # init from config
    def __init__(self, client, config):
        # self.inputs = {}
        # self.outputs = {}

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

        self.inputs = {i.name: i for i in [*self.discrete_inputs, *self.coils, *self.input_registers, *self.holding_registers]}
        self.outputs = {i.name: i for i in [*self.coils, *self.holding_registers]}

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


# DISCRETE / COIL IO
class DiscreteRangeReader:
    def __init__(self, inputs):
        _inputs = inputs
        _inputs.sort(key=lambda e: e.address)

        self.start_address = _inputs[0].address
        self.read_count = _inputs[-1].address - self.start_address + math.ceil(_inputs[-1].size)

        if(len(inputs) <= 1):
            self.inputs = _inputs
        else:
            # add "padding" between non consecutive reads
            # one reduce call has to leave the list with a last element being NO "padder"
            # padding is allways only to be inserted BEFORE the currently handeled element!
            def add_padding(acc, i):
                d_address = i.address - acc[-1].address
                return [*acc, i] if d_address == 1 else [*acc, DiscretePadder(d_address - 1), i]
            
            # start reduce operation in a defined state with the first element already added
            self.inputs = functools.reduce(add_padding, _inputs[1:], [_inputs[0]])

    def _decode(self, result):
        if(not result.isError()):
            decoder = DiscreteDecoder(result.bits)
            for input in self.inputs:
                input.decode(decoder)
        else:
            # TODO: change logging
            print("reading failed...")

    def read(self, client, unit):
        result = client.read_discrete_inputs(self.start_address, self.read_count, unit=unit)
        self._decode(result)


class CoilRangeReader(DiscreteRangeReader):
    def read(self, client, unit):
        result = client.read_coils(self.start_address, self.read_count, unit=unit)
        self._decode(result)


class DiscreteDecoder:
    def __init__(self, states):
        self.states = states
        self.pointer = 0
    
    def decode(self):
        self.pointer += 1
        return self.states[self.pointer - 1]
    
    def reset(self):
        self.pointer = 0

    def skip(self, count):
        self.pointer += count


class DiscretePadder:
    def __init__(self, count):
        self.padding = int(count)
    
    def decode(self, decoder):
        decoder.skip(self.padding)


class DiscreteInputMapping:
    def __init__(self, **kwargs):
        self.name = kwargs['name']
        self.address = kwargs['address']
        self.value = None
        self.size = 1

    def callback(self):
        # TODO: callback
        print(self.name, "is:", self.value)

    def decode(self, decoder):
        self.value = decoder.decode()
        self.callback()


class CoilMapping(DiscreteInputMapping):
    def write(self, client, value, unit):
        print(self.name, "writing:", value)
        client.write_coil(self.address, value, unit=unit)


# REGISTER IO
class InputRegisterRangeReader:
    def __init__(self, inputs, byteorder, wordorder):
        _inputs = inputs
        _inputs.sort(key=lambda e: e.address + 0.5*e.offset)

        self.byteorder = byteorder
        self.wordorder = wordorder

        self.start_address = _inputs[0].address
        self.read_count = _inputs[-1].address - self.start_address + math.ceil(_inputs[-1].size)

        if(len(inputs) <= 1):
            self.inputs = _inputs
        else:
            # add "padding" between non consecutive reads
            # one reduce call has to leave the list with a last element being NO "padder"
            # padding is allways only to be inserted BEFORE the currently handeled element!
            def add_padding(acc, i):
                d_address = (i.address + i.offset * 0.5) - (acc[-1].address + acc[-1].offset * 0.5 + acc[-1].size)
                return [*acc, i] if d_address == 0 else [*acc, RegisterPadder(d_address), i]
            
            # start reduce operation in a defined state with the first element and required padding already added
            self.inputs = functools.reduce(
                add_padding, _inputs[1:],
                [_inputs[0]] if not _inputs[0].offset else [RegisterPadder(0.5), _inputs[0]]
            )
        # print(list(map(lambda e: e.name if hasattr(e, 'name') else 'padder', self.inputs)))

    def _decode(self, result):
        if(not result.isError()):
            decoder = PLCPayloadDecoder.fromRegisters(result.registers, byteorder=self.byteorder, wordorder=self.wordorder)
            for input in self.inputs:
                input.decode(decoder)
        else:
            # TODO: change logging
            print("reading failed...")

    def read(self, client, unit):
        result = client.read_input_registers(self.start_address, self.read_count, unit=unit)
        self._decode(result)


class HoldingRegisterRangeReader(InputRegisterRangeReader):
    def read(self, client, unit):
        result = client.read_holding_registers(self.start_address, self.read_count, unit=unit)
        self._decode(result)


class PLCPayloadBuilder(BinaryPayloadBuilder):
    def __swap_sstring_bytes(self, bytestring):
        return b''.join([bytes(list(bytestring)[s:s+2][::-1]) for s in range(0,len(bytestring),2)])

    def add_string(self, value, **kwargs):
        # encode
        value = value.encode("utf-8")
        # pad with zero if necessary
        if(len(value)%2 and not kwargs.get('ignore_byteorder', False)):
            value += b'\x00'

        # respect byteorder
        if(not kwargs.get('ignore_byteorder', False)):
            value = self.__swap_sstring_bytes(value)

        fstring = self._byteorder + str(len(value)) + 's'
        self._payload.append(pack(fstring, value))

    def add_word(self, value):
        value = value + [False]*(16 - len(value))
        slice_direction = 1 if self._byteorder == Endian.Little else -1
        self.add_bits(value[::slice_direction][0:8][::slice_direction])
        self.add_bits(value[::slice_direction][8:16][::slice_direction])

    def __add_multi_word(self, value, wordcount):
        value = value + [False]*(16*wordcount - len(value))

        slice_direction = 1 if self._wordorder == Endian.Little else -1
        for w in range(wordcount):
            self.add_word(value[::slice_direction][w*16:(w+1)*16][::slice_direction])

    def add_dword(self, value):
        __wordcount = 2
        self.__add_multi_word(value, __wordcount)

    def add_lword(self, value):
        __wordcount = 4
        self.__add_multi_word(value, __wordcount)


class PLCPayloadDecoder(BinaryPayloadDecoder):
    def decode_byte(self):
        return self.decode_bits()
    
    def decode_word(self):
        # order bits depending on byteorder
        slice_direction = 1 if self._byteorder == Endian.Little else -1
        # when little endian, bytes are in right order and the list can be concatenated directly
        # when byteoder is big, both lists from the decode_bits operation are reversed, concatenated
        # and the resulting list reversed again
        return [*(self.decode_byte()[::slice_direction]), *(self.decode_byte()[::slice_direction])][::slice_direction]
    
    def __decode_multi_word(self, wordcount):
        # order words/registers depending on wordorder
        slice_direction = 1 if self._wordorder == Endian.Little else -1
        words = [(self.decode_word()[::slice_direction]) for _ in range(wordcount)][::slice_direction]
        return functools.reduce(lambda acc, w: acc + w, words, [])
    
    def decode_dword(self):
        __wordcount = 2
        return self.__decode_multi_word(__wordcount)
    
    def decode_lword(self):
        __wordcount = 4
        return self.__decode_multi_word(__wordcount)
    
    # expects string to be of even length (%2 == 0)!
    def __swap_sstring_bytes(self, bytestring):
        return b''.join([bytes(list(bytestring)[s:s+2][::-1]) for s in range(0,len(bytestring),2)])

    def __trim_decode_bytestring(self, bytestring, encoding):
        term_idx = bytestring.find(b'\x00')
        end = len(bytestring) if term_idx == -1 else term_idx
        return bytestring[0:end].decode(encoding)

    def decode_string(self, size=1, **kwargs):
        self._pointer += size
        s = self._payload[self._pointer - size:self._pointer]

        # swap string element pairs
        if(not kwargs.get('ignore_byteorder', False) and size > 1):
            s = self.__swap_sstring_bytes(s)
        
        return self.__trim_decode_bytestring(s, kwargs.get('encoding', 'utf-8'))

    # def decode_wstring(self, size=1, **kwargs):
    #     size *= 2 # read in "words" not in bytes
    #     self._pointer += size
    #     s = self._payload[self._pointer - size:self._pointer]

    #     # swap string element pairs
    #     s = self.__swap_sstring_bytes(s)
        
    #     return self.__trim_decode_bytestring(s, kwargs.get('encoding', 'utf-16'))


class RegisterTypeMapping:
    mapping = {
        "BYTE": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_byte(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_byte(value),
            "size": lambda **kwargs: 0.5
        },
        "WORD":     {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_word(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_word(value),
            "size": lambda **kwargs: 1
        },
        "DWORD": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_dword(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_dword(value),
            "size": lambda **kwargs: 2
        },
        "LWORD": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_lword(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_lword(value),
            "size": lambda **kwargs: 4
        },
        "SINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_8bit_int(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_8bit_int(value),
            "size": lambda **kwargs: 0.5
        },
        "INT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_16bit_int(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_16bit_int(value),
            "size": lambda **kwargs: 1
        },
        "DINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_32bit_int(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_32bit_int(value),
            "size": lambda **kwargs: 2
        },
        "LINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_64bit_int(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_64bit_int(value),
            "size": lambda **kwargs: 4
        },
        "USINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_8bit_uint(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_8bit_uint(value),
            "size": lambda **kwargs: 0.5
        },
        "UINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_16bit_uint(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_16bit_uint(value),
            "size": lambda **kwargs: 1
        },
        "UDINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_32bit_uint(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_32bit_uint(value),
            "size": lambda **kwargs: 2
        },
        "ULINT": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_64bit_uint(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_64bit_uint(value),
            "size": lambda **kwargs: 4
        },
        "REAL": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_32bit_float(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_32bit_float(value),
            "size": lambda **kwargs: 2
        },
        "LREAL": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_64bit_float(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_64bit_float(value),
            "size": lambda **kwargs: 4
        },
        "CHAR": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_string(),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_string(value),
            "size": lambda **kwargs: 0.5
        },
        "STRING": {
            "decoder": lambda **kwargs: lambda decoder: decoder.decode_string(RegisterTypeMapping.fix_string_alignment(**kwargs)['length'], **kwargs),
            "encoder": lambda **kwargs: lambda encoder, value: encoder.add_string(value, **kwargs),
            "size": lambda **kwargs: RegisterTypeMapping.fix_string_alignment(**kwargs)['length'] * 0.5
        },
        # "WCHAR": {
        #     "decoder": lambda **kwargs: lambda decoder: decoder.decode_string(),
        #     "size": lambda **kwargs: 0.5
        # },
        # "WSTRING": {
        #     "decoder": lambda **kwargs: lambda decoder: decoder.decode_wstring(kwargs['length'], **kwargs),
        #     "size": lambda **kwargs: kwargs['length']
        # }
    }

    @staticmethod
    def fix_string_alignment(**kwargs):
        if(not kwargs.get('ignore_byteorder', False)):
            kwargs = {**kwargs, 'length': kwargs['length'] + kwargs['length']%2}
        return kwargs

    @staticmethod
    def get_size(type, **kwargs):
        return RegisterTypeMapping.mapping[type.upper()]['size'](**kwargs)
    
    @staticmethod
    def get_decode_fun(type, **kwargs):
        return RegisterTypeMapping.mapping[type.upper()]['decoder'](**kwargs)

    @staticmethod
    def get_encode_fun(type, **kwargs):
        return RegisterTypeMapping.mapping[type.upper()]['encoder'](**kwargs)


class InputRegisterMapping:
    def __init__(self, **kwargs):
        self.name = kwargs['name']
        self.type = kwargs['type']
        self.address = kwargs['address']
        self.value = None

        self.size = RegisterTypeMapping.get_size(**kwargs)
        # build offset flag for default big endian byteorder, XOR with not byteorder == Endian.Big
        # when offset == true, the byte is the second byte read from the register, requiring padding
        # before reading, when it's the only byte to be read from a register
        if(self.size == 0.5):
            self.offset = (not kwargs['byteorder'] == Endian.Big) ^ (True if 'LOW' in kwargs.get('offset', '').upper() else False)
        else:
            self.offset = False

        self.decode_from = RegisterTypeMapping.get_decode_fun(**kwargs)

    def callback(self):
        # TODO: callback
        print(self.name, "is:", self.value)

    def decode(self, decoder):
        self.value = self.decode_from(decoder)
        self.callback()

class HoldingRegisterMapping(InputRegisterMapping):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.byteorder = kwargs['byteorder']
        self.wordorder = kwargs['wordorder']
        self.encode_to = RegisterTypeMapping.get_encode_fun(**kwargs)

    def write(self, client, value, unit):
        if(self.size == 0.5):
            print("writing to 8 bit values is currently not supported!")
            return

        encoder = PLCPayloadBuilder(byteorder=self.byteorder, wordorder=self.wordorder)
        self.encode_to(encoder, value)
        client.write_registers(self.address, encoder.to_registers())
        print(self.name, "writing:", value)

class RegisterPadder:
    # count equals registers (not bytes)
    # to skip one byte, init with 0.5
    def __init__(self, count):
        self.padding = int(2 * count)
    
    def decode(self, decoder):
        decoder.skip_bytes(self.padding)

####################################
# MODBUS SLAVE
class ModbusSlave:
    def __init__(self, config):
        # descriptor/name for external registration and multi slave id-ing
        self.name = config['name']

        # connection settings
        self.address = config['address']
        self.port = config.get('port', 502)
        self.unit = config.get('unit', 0x01)
        self.timeout = config.get('timeout_secs', 3)

        # make client connection
        self.client = ModbusClient(self.address, port=self.port, timeout=self.timeout)

        self.io_mgr = IOManager(self.client, {**config, "unit": self.unit})

    def connect(self):
        self.client.connect()
    
    def read(self):
        self.io_mgr.read()

    def write(self, mapping, value):
        self.io_mgr.write(mapping, value)


# run standalone
if __name__ == "__main__":
    with open("./slave.json") as config_file:
        config = json.load(config_file)

        slave = ModbusSlave(config)
        slave.connect()

        gripper_state = False
        str_templates = ["abcdefg", "fizzbar"]

        while True:
            slave.read()
            time.sleep(0.5)
            slave.write("gripper", gripper_state)
            slave.write("STRING", str_templates[gripper_state])
            slave.write("UINT", 42100)
            slave.write("DINT", -2000000000)
            slave.write("WORD", [True, False, True, False, True, False, False, False, False, False, False, False, True, True, True, False])
            slave.write("DWORD", [True, True, True, False, False, True, False, False, True, False, True, False, True, True, False, True, False, False, True, True, False, False, True, True, False, True, False, True, False, True, False, False])
            gripper_state = not gripper_state
    
