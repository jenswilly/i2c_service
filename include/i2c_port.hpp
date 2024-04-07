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

#ifndef I2C_PORT_H
#define I2C_PORT_H

#include <stdint.h>

namespace i2c_port
{
	class I2cPort
	{
	  public:
		I2cPort( int i2cBus, uint8_t address, bool open = true );
		~I2cPort();
		bool isOpen();

		/**
		 * @brief i2c_port::closePort Closes the port if open. Does nothing if the port is not open.
		 * @return A Boolean specifying whether the operation completed successfully or not.
		 */
		void closePort();

		/**
		* @brief Read one byte register
		* @param readRegister Register to read
		* @return Value of the register
		*/
		uint8_t readRegisterByte( uint8_t readRegister );

		/**
		* @brief Write single byte to the specified register.
		* @param writeRegister Register to write to
		* @param writeValue Value to write
		* @return
		*/
		uint8_t writeRegisterByte( uint8_t writeRegister, uint8_t writeValue );

		/**
		* @brief Write single byte.
		* @param writeValue Data to write
		* @return
		*/
		uint8_t writeByte( uint8_t writeValue );


	  private:
		int i2cFileDescriptor_ = -1;         // File Descriptor of the I2C bus. -1 means not connected.
		int error_; // Last error number from IO
		uint8_t address_;
		int i2cBus_; // I2C bus to use (set in initializer, defaults to 1)
		bool openPort();
	};

}

#endif // I2C_PORT_H