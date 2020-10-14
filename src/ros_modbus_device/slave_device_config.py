import sys
import argparse
import yaml

import logging
logger = logging.getLogger(__name__)

import rospy


class ROSModbusSlaveDeviceConfig:
    defaults = {
        'port': 502,
        'unit': 1,
        'rate': 10,
        'timeout': 3
    }

    def __init__(self, ros_modbus_slave_device):
        self.device = ros_modbus_slave_device

        # read ROS parameters and command line arguments;
        # command line arguments override ROS parameters!
        cfg_param_arg = {}
        cfg_param_arg.update({k: v for k,v in self.__load_ros_params().items() if v})
        cfg_param_arg.update({k: v for k,v in self.__load_cmdl_args().items() if v})
        
        # read config and mapping from client config file;
        # ROS parameters and command line arguments override the config
        cfg_mapping = self.__load_mapping_cfg(cfg_param_arg.get('mapping_file_path', None))

        # merge configs in asscending order of precedence:
        # DEFAULTS << CONFIG FILE << ROS PARAMS << CMD ARGS
        self.config = self.defaults.copy()
        self.config.update(cfg_mapping)
        self.config.update(cfg_param_arg)
                
    def __load_ros_params(self):
        params = {}
        params['mapping_file_path'] = rospy.get_param(self.device.ns_name('mapping'), None)

        params['name'] = rospy.get_param(self.device.ns_name('name'), None)

        params['address'] = rospy.get_param(self.device.ns_name('address'), None)
        params['port'] = rospy.get_param(self.device.ns_name('port'), None)
        params['unit'] = rospy.get_param(self.device.ns_name('unit'), None)
        params['timeout'] = rospy.get_param(self.device.ns_name('timeout'), None)

        params['rate'] = rospy.get_param(self.device.ns_name('rate'), None)

        return params

    def __load_cmdl_args(self):
        parser = argparse.ArgumentParser()

        parser.add_argument('--mapping', action="store", dest="mapping_file_path")

        parser.add_argument('--name', action="store", dest="name")

        parser.add_argument('--address', action="store", dest="address")
        parser.add_argument('--port', action="store", dest="port", type=int)
        parser.add_argument('--unit', action="store", dest="unit", type=int)
        parser.add_argument('--timeout', action="store", dest="timeout", type=int)

        parser.add_argument('--rate', action="store", dest="rate", type=int)

        return vars(parser.parse_known_args()[0])

        
    def __load_mapping_cfg(self, path):
        logger.info("trying to load config/mapping from: %s", path)
        try:
            with open(path) as file:
                logger.info("successfully opened: %s", path)
                return yaml.safe_load(file)
        except:
            logger.error("could not open or read a config/mapping file at '%s'", path)
            sys.exit(-1)
