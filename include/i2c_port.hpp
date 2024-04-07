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