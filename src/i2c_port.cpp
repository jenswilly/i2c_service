// The MIT License (MIT)

// Copyright (c) 2024 Jens Willy Johannsen <jens@jwrobotics.com>

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

#include "i2c_port.hpp"

#include <sys/ioctl.h>
#include <fcntl.h>
#include <string>
#include <unistd.h>

extern "C" {
  #include <linux/i2c.h>
  #include <linux/i2c-dev.h>
  #include <i2c/smbus.h> // From libi2c-dev
}

namespace i2c_port
{
	I2cPort::I2cPort( int i2cBus, uint8_t address, bool open ) :
		address_( address ),
		i2cBus_( i2cBus )
	{
		if( open )
			openPort();
	}

	I2cPort::~I2cPort()
	{
		// Close port if open
		closePort();
	}


	bool I2cPort::openPort()
	{
		char fileNameBuffer[32];
		sprintf( fileNameBuffer,"/dev/i2c-%d", i2cBus_ );
		i2cFileDescriptor_ = open( fileNameBuffer, O_RDWR );
		if( i2cFileDescriptor_ < 0 ) {
			// Could not open the file
			error_ = errno;
			return false;
		}
		if( ioctl( i2cFileDescriptor_, I2C_SLAVE, address_ ) < 0 )
		{
			// Could not open the device on the bus
			error_ = errno;
			return false;
		}
		return true;
	}

	bool I2cPort::isOpen()
	{
		return (i2cFileDescriptor_ > 0);
	}

	uint8_t I2cPort::readRegisterByte( uint8_t readRegister )
	{
		int value = i2c_smbus_read_byte_data( i2cFileDescriptor_, readRegister );
		usleep( 1000 );
		if( value < 0 )
		{
			error_ = errno;
			value = -1;
		}
		return value;
	}

	uint8_t I2cPort::writeRegisterByte( uint8_t writeRegister, uint8_t writeValue )
	{
		int value = i2c_smbus_write_byte_data( i2cFileDescriptor_, writeRegister, writeValue );
		// Wait a little bit to make sure it settles
		usleep( 10000 );        // TODO: is this necessary? -JWJ
		if( value < 0 )
		{
			error_ = errno;
			value = -1;
		}
		return value;
	}

	uint8_t I2cPort::writeByte( uint8_t writeValue )
	{
		int value = i2c_smbus_write_byte( i2cFileDescriptor_, writeValue );
		// Wait a little bit to make sure it settles
		usleep( 10000 );        // TODO: is this necessary? -JWJ
		if( value < 0 )
		{
			error_ = errno;
			value = -1;
		}
		return value;
	}

	void I2cPort::closePort()
	{
		if( i2cFileDescriptor_ > 0 ) {
			::close( i2cFileDescriptor_ );
			// WARNING - This is not quite right, need to check for error first
			i2cFileDescriptor_ = -1;
		}
	}
}