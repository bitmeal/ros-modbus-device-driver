# ROS Modbus device driver
> :pushpin: Companion paper
> * DOI: [10.1109/ETFA45728.2021.9613662](https://doi.org/10.1109/ETFA45728.2021.9613662)
> * arXiv: [arXiv:2112.11102](https://arxiv.org/abs/2112.11102)

MODBUS TCP is understood by a wealth of industrial devices from PLCs to bus couplers. Building a simple interface from ROS to these devices, allows the efficient use of available industrial hardware without any physical interfacing issues; thus allowing for much faster prototyping and development.

This package allows you to configure all inputs and outputs of your bus coupler, exposed variables of you PLC, *[other device]*, for use in ROS; just specify their coil or register address and the matching data type in a JSON file. All device inputs will be published to individual topics with a user specified name. All outputs may be written by publishing to an automatically mapped topic as well.

*For easy configuration, sensible defaults - in accordance with the MODBUS specification [1] and tested against MODBUS implementations of PLCs/bus couplers - are used, and all supported data types are in accordance with **IEC 61131-3** [2].*

> :pushpin: What are the limitations?
* :zap: Only modbus **slaves** are supported, but this should cover 95% of the use cases (the node implements a MODBUS master)
* :zap: 8 bit wide values can not be written to; they can be decoded tough - even two individual ones from one register
* :zap: No overlapping reads, or multiple reads from the same registers or coils; e.g. reading a *DWORD @ Reg. 0* (Reg. [0..1]) and the second register again as a *WORD @ Reg. 1*
* :zap: No support for arrays as single read/write/pub/sub operation
* :zap: Only *Elementary Types* from [2], with the following exceptions are supported: `BOOL`(only as coil/discrete input), **all** *duration* and *time* types, `WCHAR`/`WSTRING`


## Using the node
### Building
Install dependencies using `rosdep install -y --from-paths ros-modbus-device-driver` and build the node using `catkin build` from *catkin-tools*; **`catkin_make` is NOT supported**! Install *catkin-tools* using `pip install -U catkin_tools`, `apt install python-catkin-tools` for ROS versions up to *melodic*, `apt install python3-catkin-tools python3-osrf-pycommon` for ROS *noetic* and onwards, or see [documentation](https://catkin-tools.readthedocs.io/en/latest/installing.html) for your system.
### Running
```bash
rosrun ros_modbus_device_driver modbus_device_driver.py _mapping:=devicemapping.json
```
1. :page_facing_up: define a device and its mapping (see below)
2. :computer: run the driver node, using `rosrun` and provide the path to your config file, either as command line argument `--mapping <path>`, or ROS parameter `_mapping:=<path>`
3. :tada: *profit*

#### ROS parameters and command line arguments
All [connection parameters](#connection-parameters) are exposed as *private* (namespaced) ROS parameters, and command line arguments. Apart from using ROS parameters in a `.launch` file, they can be passed on the command line in the form of `_parameter:=value`. Traditional command line arguments have to be supplied like `--parameter value`. To run multiple driver instances for multiple divices of the same type, use the `name` parameter, assigning a unique name per device/driver instance. Available parameters/arguments:
| ROS param | :computer: command line argument |
|---|---|
| `name`        | `--name`      |
| `address`	    | `--address`	|
| `port`	    | `--port`	    |
| `unit`	    | `--unit`	    |
| `timeout`	    | `--timeout`	|
| `rate`	    | `--rate`	    |

> :pushpin: Why is only this subset of options configurable on the command line or through ROS parameters?
1. You **need** a device mapping/config file anyways; the IO configuration through parameters and arguments is simply not feasible.
2. A mapping is created per specific device. Limiting the startup parameters, forces you to create a sane configuration file, that may be used for **any instance** of the mapped device. *Neat, is it?*
3. *Effectively see 2.*: The connection parameters and your timing requirements may vary between two instances of the same device, as well as the descriptor/name you want to use. All other configuration options are specific to the device and there is no point in changing e.g. your byte order at startup.

### Interfacing
The node automatically maps your IOs to topics in the way shown below. `<name>` is read from the device config (`"name"`), the command line `--name` or a ROS parameter `name` and represents the namespace of your MODBUS device (the running node itself uses an anonymous namespace). `<mappin-name>` is the name given to an IO in your device mapping.

| direction | topic | mapping type |
|---|---|---|
| reading | `/<name>/<mapping-name>` | *coils, discrete inputs,* all *registers* |
| writing | `/<name>/<mapping-name>/write` | *coils, holding registers* |


## Slave device definition
Define a slave device to interact with, as a JSON object, by giving it a `name`-property and configuring its connection parameters and `mapping`s for:
* Coils [`coils`] (digital I/O *rw*) by assigning an address-value to a key as its identifier
* Discrete inputs [`discrete_inputs`] (digital IN *ro*) like the coils
* Holding registers [`holding_registers`] (analog I/O, process data *rw*)
* Input registers [`input_registers`] (analog IN, process data *ro*) like the holding registers
*for the used terminology, refer to [1] section 4.3*

### Connection parameters
Configure the connection to your slave-device (example below):
* IP address: `address`
* Port: `port`; default=`502`
* MODBUS unit ID: `unit`; default=`0x01`
* Connection/read timeout: `timeout` (in seconds); default=`3`
* Rate to poll for changes as `rate` (in [*Hz*]); default=`10`



```jsonc
// example basic configuration
{
    "name": "mymodbusslave",

    "address": "192.168.10.101",
    "port": 502,
    "unit": 1,
    "timeout": 3,

    "rate": 20,

    // ...
}
```

### Mapping coils & discrete inputs
Coils an discrete inputs store/accept *boolean* values (single bit). Coils allow for read and write access, discrete inputs allow read access only.


They are configured as a map, using the desired names as keys and their addresses as assigned values:
```jsonc
// coils and discrete input configuration example
{
    "coils": {
        "coil_name": 1,
        // ...
    },
    "discrete_inputs": {
        "input_name": 10001,
        // ...
    }
}
```

### Mapping holding & input registers
Registers are 16 bit wide memory-"blocks" that can be read (input and holding registers) and written to (holding registers only). Registers may be configured to hold (a subset of) the data types defined in **IEC 61131-3** [2]. The types may be wider than 16 bits; in your register configuration you simply specify the start address (lowest address value occupied by a word of the value to be read). Registers are configured in a map with the desired name as key and a map with information about the stored/written type as `type` and the address as `address` as assigned value.

See sections and table below for available types and their specification.

```jsonc
// register configuration example
{
    "holding_registers": {
        "register_name": {
            "address": 30001,
            "type": "WORD"
        },
        // ...
    },
    "input_registers": {
        "input_name": {
            "address": 30002,
            "type": "BYTE",
            "offset": "high"
        },
        // ...
    },
}
```


#### 8 bit values (1B/8b)
**Two 8 bit values can be decoded from one register.** For all supported 8 bit values (`BYTE`, `SINT`, `USINT`), an `offset` has to be specified as `"low"` or `"high"`. `"high"` being the first byte transmitted when byte order is *big endian (default)*, respectively the second byte on *little endian* encoding.

**The meaning of your `offset` varies with your configured *byte order***.
```jsonc
{
    "highregister": {
        "address": 30001,
        "type": "BYTE",
        "offset": "high"
    },
    "lowregister": {
        // :pushpin: note the same address value as above 
        "address": 30001,
        "type": "BYTE",
        "offset": "low"
    }
}
```

#### Strings
Strings require an additional `length` field to be provided! Strings may only consist of 8 bit wide characters, where each 16 bit register holds two characters (*`WSTRING` is not supported for a lack PLCs to test against*). The string is read from the given starting address. Length may be an uneven number. If your device does not respect the configured byte order when transmitting string, enable `ignore_byteorder` **per every string mapping**. The encoding of the string may be specified using the `encoding` option; see [3] for valid encodings, default is *UTF-8*(`utf-8`).
```jsonc
{
    "somestring": {
        "address": 30010,
        "type": "STRING",
        "length": 8,
        "encoding": "utf-8",
        "ignore_byteorder": false
    }
}
```

#### Boolean values
*Directly decoding `BOOL` from a register is not supported. Boolean values should be mapped as coils/discrete input or read as `BYTE`, `WORD`, `DWORD`, `LWORD` and read from the resulting array of boolean values.*

#### Arrays
**not supported for now**

#### List of types
|Type       | Bits  | Registers | Representation    | Info              |
|:----------|------:|----------:|:------------------|:------------------|
| `BYTE`    |  8    | 1/2		| Bool[8]           | requires `offset` |
| `WORD`    | 16    | 1		    | Bool[16]          |                   |
| `DWORD`   | 32    | 2 		| Bool[32]          |                   |
| `LWORD`   | 64    | 4	    	| Bool[64]          |                   |
|           |                   |                   |                   |
| `SINT`    |  8    | 1/2		| Int8              | requires `offset` |
| `INT`     | 16    | 1	    	| Int16             |                   |
| `DINT`    | 32    | 2	    	| Int32             |                   |
| `LINT`    | 64    | 4	    	| Int64             |                   |
|           |                   |                   |                   |
| `USINT`   |  8    | 1/2		| UInt8             | requires `offset` |
| `UINT`    | 16    | 1		    | UInt16            |                   |
| `UDINT`   | 32    | 2		    | UInt32            |                   |
| `ULINT`   | 64    | 4		    | UInt64            |                   |
|           |       |           |                   |                   |
| `REAL`    | 32    | 2		    | Float32           |                   |
| `LREAL`   | 64    | 4		    | Float64           |                   |
|           |       |           |                   |                   |
| `CHAR`    |  8    | 1/2		| Char              | requires `offset` |
| `STRING`  |  -    | `length`/2| String            | requires `length` |


### Read optimization
When configuring coils and registers that are not continuously mapped in your slaves memory, individual read operations will be generated for all continuously mapped chunks of memory. This avoids errors when trying to read invalid addresses on the client, and helps to keep the amount of transferred data low. When your slave allows reads at the addresses between your mapped coils/inputs/registers, you can set the options `discrete_read_continuous` (effects coils and discrete inputs) or `registers_read_continuous` (effects all registers) to allow reads at unmapped addresses. This reduces the overall number of read operations by closing the gaps between mapped addresses and discarding the unused data. To further optimize the behavior, you can set a custom value for `discrete_read_separation_gap` and `register_read_separation_gap`. A gap in the mapped address space greater than these values will result in generation of individual read operations. When optimizing, remember that the amount of data for one coil/discrete input is ***1 bit*** and for a register it is ***1 Byte***; the ratio of this parameters may thus be in the range of $8/1$.

```jsonc
// controlling generation of separate read operations
{
    "discrete_read_continuous": true,
    "discrete_read_separation_gap": 64,

    "registers_read_continuous": false,
}
```


### Byte and Word order
#### Byte order
:zap: YOU *SHOULD* NEVER NEED TO CHANGE THE *BYTE ORDER* :zap:

The byte order is specified to be big endian, per section 4.2 of the modbus specification [1]. The *byte order* describes the order of the two 8 bit bytes that make up one 16 bit register. You may configure the byte order (endianness) to deviate from this specification.
```jsonc
// sets byte order: little endian
{
    "byteorder_reverse": true
}
```

#### Word order
For data wider than one register (16 bit) you can specify a word order. Default is configured to be little endian.

```jsonc
// sets word order: big endian
{
    "wordorder_reverse": true 
}
```

### Example config
Below configures a slave that will be mapped as `/mymodbusslave`, with two configured coils to read from and write to, two discrete inputs, and one holding and input register each. All bindings to topics, as `/mymodbusslave/<coil|input|register-name>/[status|write]` for reading and writing to the slave are automatically configured.
```jsonc
// modbus-slave configuration example
{
    "name": "mymodbusslave",

    "address": "192.168.10.101",
    "port": 502,
    "unit": 1,
    "timeout": 3,

    "rate": 20,

    "byteorder_reverse": false, // byte order: default, big endian
    "wordorder_reverse": false, // word order: default, little endian

    "discrete_read_continuous": true,
    "discrete_read_separation_gap": 64,

    "registers_read_continuous": false,

    "mapping": {
        "coils": {
            "gripper": 1,
            "signal": 2,
            // ...
        },
        "discrete_inputs": {
            "overload": 10001,
            "presence": 10010,
            // ...
        },
        "input_registers": {
            "distance": {
                "address": 30001,
                "type": "REAL"
            },
            // ...
        },
        "holding_registers": {
            "name": {
                "address": 40012,
                "type": "STRING",
                "length": 12
            },
            // ...
        }
    }
}
```

## TODO
* [ ] add tests
* [ ] add config/mapping schema parser; detect errors with meaningful messages before running
* [x] use YAML config file (more *ros-like*)
* [ ] translate examples to YAML
* [ ] write/output using services
* [ ] support arrays
* [ ] publish on change only
* [ ] publish on change and with rate
* [ ] configure latching & queue size
* [ ] support writing individual 8 bit values
* [ ] simple math for scaling numeric IOs (`*INT`, `*REAL`)

## Tested hardware
* **WAGO PFC100** PLC: Exposing select PLC program variables. Tested all supported data types. *Assumed to be representative for the 750-8xx series.*
* **Beckhoff BC9000** PLC, configured as simple bus coupler: Reading and writing digital in- and outputs, reading analog inputs and reading and writing registers for communication with RS-422/485 module.
* **Beckhoff BK9000** coupler: Tests as for **BC9000**.

### Hardware info
Some help for using your hardware:
#### Beckhoff
Beckhoff couplers, and PLCs (e.g. BC9000) configured accordingly, map the connected IOs in the following way. Be sure to use the right type of mapping in your configuration, as addressing depends on the requested MODBUS function.
* discrete in: starting at address 0, counting input channels in the order of attached *input* modules
* coils: starting at address 0, counting output channels in the order of attached *output* modles
* input registers: starting at address 0, counting channels in order of attached *input* modules. Inputs may consume two registers each, with the actual data register being the second one; e.g.: first analog input data is at register 1, the second at 3, ...
* holding registers: starting at address 2048, counting in order of attached *output* modules


A Beckhoff BC9000 can be transformed into a BK9000 by performing a hardware reset and setting table 2 registers 4-11 to `0x0000` using TwinCAT.

## License and citing
The source code provided in this repository is licensed under MPL 2.0. *A different license may apply to binary versions of this software!*

If used in published research, please cite as:
* A. Wendt and T. Schüppstuhl, "A Solution to the Generalized ROS Hardware IO Problem - A Generic Modbus/TCP Device Driver for PLCs, Sensors and Actuators," 2021 26th IEEE International Conference on Emerging Technologies and Factory Automation (ETFA), 2021, pp. 1-8, DOI: [10.1109/ETFA45728.2021.9613662](https://doi.org/10.1109/ETFA45728.2021.9613662).

You may use following BibTeX entry:
```bibtex
@inproceedings{Wendt_Schuppstuhl_2021_ROSIO,
    author={Wendt, Arne and Sch{\"u}ppstuhl, Thorsten},
    booktitle={{2021 26th IEEE International Conference on Emerging Technologies and Factory Automation (ETFA)}},
    title={{A Solution to the Generalized ROS Hardware IO Problem - A Generic Modbus/TCP Device Driver for PLCs, Sensors and Actuators}},
    year={2021},
    doi={10.1109/ETFA45728.2021.9613662}
}
```

## References
[1] [MODBUS Application Protocol Specification; *https://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf*](https://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf)

[2] [**IEC 61131-3** Programmable controllers - Part 3: Programming languages; *https://en.wikipedia.org/wiki/IEC_61131-3*](https://en.wikipedia.org/wiki/IEC_61131-3#Data_types)

[3] [*codecs* — Codec registry and base classes — Python 3.9.0 documentation: Standard Encodings](https://docs.python.org/3/library/codecs.html#standard-encodings)
