#include "tca9548a/tca9548a_node.hpp"

namespace tca9548a {

Tca9548aNode::Tca9548aNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("tca9548a_node", options)
{
    // 1. Declare and retrieve the I2C bus parameter
    i2c_bus_param_ = this->declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");

    // 2. Declare and retrieve the I2C address parameter (default to 0x70)
    int address_int = this->declare_parameter<int>("i2c_address", 0x70);
    // Cast the integer parameter to a uint8_t
    i2c_address_param_ = static_cast<uint8_t>(address_int);

    // 3. Initialize the driver with the retrieved parameters
    driver_ = tca9548a::Tca9548a(i2c_bus_param_, i2c_address_param_);

    RCLCPP_INFO(this->get_logger(), "Starting TCA9548A ROS 2 Node...");
    RCLCPP_INFO(this->get_logger(), "Configured with I2C bus: %s and address: 0x%02X",
                i2c_bus_param_.c_str(), i2c_address_param_);
    
    if (!driver_.open_bus()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C bus. Node will not function.");
    } else {
        RCLCPP_INFO(this->get_logger(), "I2C bus opened successfully.");
    }

    select_channel_service_ = this->create_service<tca9548a::srv::SelectChannel>(
        "select_channel",
        std::bind(&Tca9548aNode::handle_select_channel, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Service server for 'select_channel' is ready.");
}

void Tca9548aNode::handle_select_channel(
    const std::shared_ptr<tca9548a::srv::SelectChannel::Request> request,
    std::shared_ptr<tca9548a::srv::SelectChannel::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received request to select channel: %d", request->channel);

    bool success = driver_.select_channel(request->channel);
    response->success = success;

    if (success) {
        RCLCPP_INFO(this->get_logger(), "Successfully selected channel %d.", request->channel);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to select channel %d. Check the I2C bus and device.", request->channel);
    }
}

} // namespace tca9548a

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tca9548a::Tca9548aNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}