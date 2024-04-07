#include "rclcpp/rclcpp.hpp"
#include "i2c_interfaces/srv/i2_c_read_register_byte.hpp"
#include "i2c_interfaces/srv/i2_c_write_register_byte.hpp"
#include "i2c_interfaces/srv/i2_c_write_byte.hpp"

#include <memory>

void readRegisterByte(const std::shared_ptr<i2c_interfaces::srv::I2CReadRegisterByte::Request> request,
                      std::shared_ptr<i2c_interfaces::srv::I2CReadRegisterByte::Response>      response)
{
    response->success = true;
    response->value = 0;
}

void writeByte(const std::shared_ptr<i2c_interfaces::srv::I2CWriteByte::Request> request,
               std::shared_ptr<i2c_interfaces::srv::I2CWriteByte::Response>      response)
{
    response->success = true;
    response->value = 0;
}

void writeRegisterByte(const std::shared_ptr<i2c_interfaces::srv::I2CWriteRegisterByte::Request> request,
                       std::shared_ptr<i2c_interfaces::srv::I2CWriteRegisterByte::Response>      response)
{
    response->success = true;
    response->value = 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("i2c_server");

    rclcpp::Service<i2c_interfaces::srv::I2CReadRegisterByte>::SharedPtr readRegisterByteService =
        node->create_service<i2c_interfaces::srv::I2CReadRegisterByte>("i2c_read_register_byte", &readRegisterByte);
    rclcpp::Service<i2c_interfaces::srv::I2CWriteByte>::SharedPtr writeByteService =
        node->create_service<i2c_interfaces::srv::I2CWriteByte>("i2c_write_byte", &writeByte);
    rclcpp::Service<i2c_interfaces::srv::I2CWriteRegisterByte>::SharedPtr writeRegisterByteService =
        node->create_service<i2c_interfaces::srv::I2CWriteRegisterByte>("i2c_write_register_byte", &writeRegisterByte);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I2C service server ready.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}