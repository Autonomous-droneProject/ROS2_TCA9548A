#ifndef TCA9548A_TCA9548A_NODE_HPP_
#define TCA9548A_TCA9548A_NODE_HPP_

#include "tca9548a/tca9548a.hpp"
#include "tca9548a/srv/select_channel.hpp"
#include <rclcpp/rclcpp.hpp>

namespace tca9548a {

class Tca9548aNode : public rclcpp::Node {
public:
  explicit Tca9548aNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  rclcpp::Service<tca9548a::srv::SelectChannel>::SharedPtr
      select_channel_service_;
  Tca9548a driver_;

  // Store parameter values for easy access if needed
  std::string i2c_bus_param_;
  uint8_t i2c_address_param_;

  void handle_select_channel(
      const std::shared_ptr<tca9548a::srv::SelectChannel::Request> request,
      std::shared_ptr<tca9548a::srv::SelectChannel::Response> response);
};

} // namespace tca9548a

#endif // TCA9548A_TCA9548A_NODE_HPP_