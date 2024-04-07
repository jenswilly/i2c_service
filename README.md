# I2C Service

## Prerequisites

The `libi2c` needs to be installed. If not already installed, install with:

`sudo apt-get install libi2c-dev`

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

