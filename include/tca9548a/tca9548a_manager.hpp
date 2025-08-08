#ifndef TCA9548A_TCA9548A_MANAGER_HPP_
#define TCA9548A_TCA9548A_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

// Include the generated service header
#include "tca9548a/srv/select_channel.hpp"
#include "tca9548a/tca9548a.hpp"

namespace tca9548a {

class Tca9548aManager : public rclcpp::Node {
public:
  explicit Tca9548aManager(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  std::map<uint8_t, Tca9548a> tca_drivers_;
  rclcpp::Service<tca9548a::srv::SelectChannel>::SharedPtr
      select_channel_service_;

  std::string i2c_bus_param_;
  std::vector<int> tca_addresses_param_;

  void handle_select_channel(
      const std::shared_ptr<tca9548a::srv::SelectChannel::Request> request,
      std::shared_ptr<tca9548a::srv::SelectChannel::Response> response);
};

} // namespace tca9548a

#endif // TCA9548A_TCA9548A_MANAGER_HPP_