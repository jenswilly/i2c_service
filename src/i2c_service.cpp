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

#include "rclcpp/rclcpp.hpp"
#include "i2c_interfaces/srv/i2_c_read_register_byte.hpp"
#include "i2c_interfaces/srv/i2_c_write_register_byte.hpp"
#include "i2c_interfaces/srv/i2_c_write_byte.hpp"
#include "i2c_port.hpp"
#include <memory>

i2c_port::I2cPort *i2cPort_ = NULL;
int i2cBus_ = 0;

void readRegisterByte(const std::shared_ptr<i2c_interfaces::srv::I2CReadRegisterByte::Request> request,
					  std::shared_ptr<i2c_interfaces::srv::I2CReadRegisterByte::Response> response)
{
	RCLCPP_INFO(rclcpp::get_logger("i2c_service"), "Reading register %d from address 0x%02X", request->reg, request->address);

	if (i2cPort_)
	{
		RCLCPP_ERROR(rclcpp::get_logger("i2c_service"), "I2C device /dev/i2c-%d already open!", i2cBus_);
		response->success = false;
		delete i2cPort_;
		i2cPort_ = NULL;
		return;
	}

	i2cPort_ = new i2c_port::I2cPort(i2cBus_, request->address);
	if (!i2cPort_->isOpen())
	{
		RCLCPP_ERROR(rclcpp::get_logger("i2c_service"), "Unable to open I2C device /dev/i2c-%d connection to device 0x%02X", i2cBus_, request->address);
		response->success = false;
		delete i2cPort_;
		i2cPort_ = NULL;
	}

	uint8_t value = i2cPort_->readRegisterByte(request->reg);
	response->success = true;
	response->value = value;

	i2cPort_->closePort();
	delete i2cPort_;
	i2cPort_ = NULL;
}

void writeByte(const std::shared_ptr<i2c_interfaces::srv::I2CWriteByte::Request> request,
			   std::shared_ptr<i2c_interfaces::srv::I2CWriteByte::Response> response)
{
	if (i2cPort_)
	{
		RCLCPP_ERROR(rclcpp::get_logger("i2c_service"), "I2C device /dev/i2c-%d already open!", i2cBus_);
		response->success = false;
		delete i2cPort_;
		i2cPort_ = NULL;
	}

	i2cPort_ = new i2c_port::I2cPort(i2cBus_, request->address);
	if (!i2cPort_->isOpen())
	{
		RCLCPP_ERROR(rclcpp::get_logger("i2c_service"), "Unable to open I2C device /dev/i2c-%d connection to device 0x%02X", i2cBus_, request->address);
		response->success = false;
		delete i2cPort_;
		i2cPort_ = NULL;
	}

	uint8_t value = i2cPort_->writeByte(request->value);
	response->success = true;
	response->value = value;

	delete i2cPort_;
	i2cPort_ = NULL;
}

void writeRegisterByte(const std::shared_ptr<i2c_interfaces::srv::I2CWriteRegisterByte::Request> request,
					   std::shared_ptr<i2c_interfaces::srv::I2CWriteRegisterByte::Response> response)
{
	if (i2cPort_)
	{
		RCLCPP_ERROR(rclcpp::get_logger("i2c_service"), "I2C device /dev/i2c-%d already open!", i2cBus_);
		response->success = false;
		delete i2cPort_;
		i2cPort_ = NULL;
	}

	i2cPort_ = new i2c_port::I2cPort(i2cBus_, request->address);
	if (!i2cPort_->isOpen())
	{
		RCLCPP_ERROR(rclcpp::get_logger("i2c_service"), "Unable to open I2C device /dev/i2c-%d connection to device 0x%02X", i2cBus_, request->address);
		response->success = false;
		delete i2cPort_;
		i2cPort_ = NULL;
	}

	uint8_t value = i2cPort_->writeRegisterByte(request->reg, request->value);
	response->success = true;
	response->value = value;

	delete i2cPort_;
	i2cPort_ = NULL;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("i2c_server");

	// Read parameters. Use postfix of I2C bus. E.g. /dev/i2c-1 -> i2c_bus=1
	node->declare_parameter("i2c_bus", 0);
	node->get_parameter("i2c_bus", i2cBus_);

	rclcpp::Service<i2c_interfaces::srv::I2CReadRegisterByte>::SharedPtr readRegisterByteService =
		node->create_service<i2c_interfaces::srv::I2CReadRegisterByte>("i2c_read_register_byte", &readRegisterByte);
	rclcpp::Service<i2c_interfaces::srv::I2CWriteByte>::SharedPtr writeByteService =
		node->create_service<i2c_interfaces::srv::I2CWriteByte>("i2c_write_byte", &writeByte);
	rclcpp::Service<i2c_interfaces::srv::I2CWriteRegisterByte>::SharedPtr writeRegisterByteService =
		node->create_service<i2c_interfaces::srv::I2CWriteRegisterByte>("i2c_write_register_byte", &writeRegisterByte);

	RCLCPP_INFO(rclcpp::get_logger("i2c_service"), "I2C service server ready using /dev/i2c-%d.", i2cBus_);

	rclcpp::spin(node);
	rclcpp::shutdown();
}