import logging
logger = logging.getLogger(__name__)


class DiscreteInputMapping:
    def __init__(self, **kwargs):
        self.name = kwargs['name']
        self.address = kwargs['address']
        self.value = None
        self.size = 1

        self.callbacks = []

    def attach_callback(self, method):
        self.callbacks.append(method)

    def fire_callbacks(self):
        for f in self.callbacks:
            f(self.value)

    def decode(self, decoder):
        self.value = decoder.decode()
        self.fire_callbacks()


class CoilMapping(DiscreteInputMapping):
    def write(self, client, value, unit):
        logger.debug("%s: writing %s", self.name, str(value))
        client.write_coil(self.address, value, unit=unit)

class DiscretePadder:
    def __init__(self, count):
        self.padding = int(count)
    
    def decode(self, decoder):
        decoder.skip(self.padding)
