#ifndef TCA9548A_TCA9548A_MANAGER_HPP_
#define TCA9548A_TCA9548A_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

// Include the generated service header
#include "tca9548a/srv/select_channel.hpp"

namespace tca9548a {

class Tca9548aManager : public rclcpp::Node {
public:
  explicit Tca9548aManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // Use a vector to hold a client for each managed TCA9548A device
  std::vector<rclcpp::Client<tca9548a::srv::SelectChannel>::SharedPtr> clients_;

  // Timer to periodically trigger service calls
  rclcpp::TimerBase::SharedPtr timer_;

  // A counter to cycle through the channels or clients
  size_t channel_counter_ = 0;

  // Callback function for the timer
  void timer_callback();

  // Helper function to call the service asynchronously
  void call_select_channel_service(size_t client_index, uint8_t channel);
};

} // namespace tca9548a

#endif // TCA9548A_TCA9548A_MANAGER_HPP_