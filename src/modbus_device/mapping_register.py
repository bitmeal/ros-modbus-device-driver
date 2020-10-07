import logging
logger = logging.getLogger(__name__)

from modbus_device.mapping_register_IEC_61131_3_types import RegisterTypeMapping
from modbus_device.payload_register import IECPayloadBuilder

from pymodbus.constants import Endian


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

        self.callbacks = []

    def attach_callback(self, method):
        self.callbacks.append(method)

    def fire_callbacks(self):
        for f in self.callbacks:
            f(self.value)

    def decode(self, decoder):
        self.value = self.decode_from(decoder)
        self.fire_callbacks()


class HoldingRegisterMapping(InputRegisterMapping):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.byteorder = kwargs['byteorder']
        self.wordorder = kwargs['wordorder']
        self.encode_to = RegisterTypeMapping.get_encode_fun(**kwargs)

    def write(self, client, value, unit):
        if(self.size == 0.5):
            logger.error("%s: writing an 8 bit value like (%s) is not currently supported!", self.name, self.type)
            return

        encoder = IECPayloadBuilder(byteorder=self.byteorder, wordorder=self.wordorder)
        self.encode_to(encoder, value)
        client.write_registers(self.address, encoder.to_registers())
        logger.debug("%s: writing %s", self.name, str(value))


class RegisterPadder:
    # count equals registers (not bytes)
    # to skip one byte, init with 0.5
    def __init__(self, count):
        self.padding = int(2 * count)
    
    def decode(self, decoder):
        decoder.skip_bytes(self.padding)
