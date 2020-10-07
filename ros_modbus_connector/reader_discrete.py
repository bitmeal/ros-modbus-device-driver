from ros_modbus_connector.payload_discrete import DiscreteDecoder
from ros_modbus_connector.mapping_discrete import DiscretePadder

import functools
import math


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

