#include "rclcpp/rclcpp.hpp"
#include "tca9548a/tca9548a.hpp" // Adjusted include path
#include "tca9548a/srv/select_channel.hpp" // Adjusted include path for the service

#include <memory>

class TcaManagerNode : public rclcpp::Node
{
public:
  TcaManagerNode() : Node("tca_manager_node"), current_channel_(-1)
  {
    this->declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
    this->declare_parameter<int>("tca_address", 0x70);

    std::string i2c_bus = this->get_parameter("i2c_bus").as_string();
    uint8_t tca_address = this->get_parameter("tca_address").as_int();

    multiplexer_ = std::make_unique<Tca9548a>(i2c_bus, tca_address);
    if (!multiplexer_->open_bus()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open I2C bus on %s. Shutting down.", i2c_bus.c_str());
      rclcpp::shutdown();
      return;
    }

    auto service_callback = std::bind(&TcaManagerNode::handle_select_channel, this, std::placeholders::_1, std::placeholders::_2);
    service_ = this->create_service<tca9548a::srv::SelectChannel>("/select_channel", service_callback);

    RCLCPP_INFO(this->get_logger(), "TCA9548A Manager is ready and providing '/select_channel' service.");
  }

private:
  void handle_select_channel(
    const std::shared_ptr<tca9548a::srv::SelectChannel::Request> request,
    std::shared_ptr<tca9548a::srv::SelectChannel::Response> response)
  {
    int requested_channel = request->channel;

    if (requested_channel < 0 || requested_channel > 7) {
      response->success = false;
      response->message = "Invalid channel. Must be between 0 and 7.";
      RCLCPP_WARN(this->get_logger(), "Received request for invalid channel: %d", requested_channel);
      return;
    }
    
    if (requested_channel == current_channel_) {
      response->success = true;
      response->message = "Channel " + std::to_string(requested_channel) + " was already selected.";
      return;
    }

    if (multiplexer_->select_channel(requested_channel)) {
      response->success = true;
      response->message = "Successfully selected channel " + std::to_string(requested_channel);
      RCLCPP_INFO(this->get_logger(), "Switched to channel %d", requested_channel);
      current_channel_ = requested_channel;
    } else {
      response->success = false;
      response->message = "Failed to select channel " + std::to_string(requested_channel) + " via I2C write.";
      RCLCPP_ERROR(this->get_logger(), "I2C write failed for channel %d", requested_channel);
    }
  }

  rclcpp::Service<tca9548a::srv::SelectChannel>::SharedPtr service_;
  std::unique_ptr<Tca9548a> multiplexer_;
  int8_t current_channel_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcaManagerNode>());
  rclcpp::shutdown();
  return 0;
}