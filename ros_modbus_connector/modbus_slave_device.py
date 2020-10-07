from ros_modbus_connector.io_manager import IOManager

from pymodbus.client.sync import ModbusTcpClient as ModbusClient


class ModbusSlaveDevice:
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

        self.inputs = self.io_mgr.inputs
        self.outputs = self.io_mgr.outputs

    def connect(self):
        self.client.connect()
    
    def read(self):
        self.io_mgr.read()

    def write(self, mapping, value):
        self.io_mgr.write(mapping, value)
    
    def attach_callbacks_discrete(self, handler_function):
        self.io_mgr.attach_callbacks_discrete(handler_function)

    def attach_callbacks_registers(self, handler_function):
        self.io_mgr.attach_callbacks_registers(handler_function)

    def attach_callbacks(self, handler_function):
        self.io_mgr.attach_callbacks(handler_function)



