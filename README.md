# I2C Service

## Prerequisites

The `libi2c` and `i2c-tools` needs to be installed. If not already installed, install with:

```bash
sudo apt-get install i2c-tools
sudo apt-get install libi2c-dev
```

# Services
## I2CReadRegisterByte

Read a single byte from a specified register

### Request/response
```
i2c_interfaces/srv/I2CReadRegisterByte

int8 address
uint8 reg
---
bool success
uint8 value
```

## I2CWriteByte
...

## I2CWriteRegisterByte

# MCP23017

This package was developed to control a [MCP23017](https://www.microchip.com/en-us/product/mcp23017) GPIO expander on a custom auxiliary interface board with some mounted LEDs.

This section shows how to control the LEDs connected to an MCP23017 on the AIB.

### Setup
WriteRegisterByte

```
i2c_write_srv.request.address = 0x20;
i2c_write_srv.request.reg = 0x00;
i2c_write_srv.request.value = 0x00;
```

### Update LEDs
```
#define LED_RED 0x01
#define LED_GREEN 0x02
#define LED_ORANGE 0x04
#define LED_BLUE 0x08

uint8_t ledValue_ = LED_BLUE | LED_ORANGE;

i2c_service::I2CWriteRegisterByte i2c_write_srv;
i2c_write_srv.request.address = 0x20;
i2c_write_srv.request.reg = 0x12;
i2c_write_srv.request.value = ledValue_;
```

### Command line
```bash
ros2 run i2c_service i2c_service

ros2 service call /i2c_write_register_byte i2c_interfaces/srv/I2CWriteRegisterByte "{address: 32, reg: 0, value: 0}"

ros2 service call /i2c_write_register_byte i2c_interfaces/srv/I2CWriteRegisterByte "{address: 32, reg: 18, value: 12}"
```