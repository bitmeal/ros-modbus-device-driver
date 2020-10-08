import logging
logger = logging.getLogger(__name__)

from modbus_device.mapping_register import RegisterPadder
from modbus_device.payload_register import IECPayloadDecoder

import functools
import math


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
                # return [*acc, i] if d_address == 0 else [*acc, RegisterPadder(d_address), i] #PYTHON3
                return (acc + [i]) if d_address == 0 else (acc + [RegisterPadder(d_address), i])
            
            # start reduce operation in a defined state with the first element and required padding already added
            self.inputs = functools.reduce(
                add_padding, _inputs[1:],
                [_inputs[0]] if not _inputs[0].offset else [RegisterPadder(0.5), _inputs[0]]
            )

    def _decode(self, result):
        if(not result.isError()):
            decoder = IECPayloadDecoder.fromRegisters(result.registers, byteorder=self.byteorder, wordorder=self.wordorder)
            for input in self.inputs:
                input.decode(decoder)
        else:
            logger.warning("reading registers at addresses [%d..%d] failed!", self.start_address, self.start_address + self.read_count - 1)

    def read(self, client, unit):
        result = client.read_input_registers(self.start_address, self.read_count, unit=unit)
        self._decode(result)


class HoldingRegisterRangeReader(InputRegisterRangeReader):
    def read(self, client, unit):
        result = client.read_holding_registers(self.start_address, self.read_count, unit=unit)
        self._decode(result)

