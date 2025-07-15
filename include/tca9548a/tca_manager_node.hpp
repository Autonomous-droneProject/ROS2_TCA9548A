#ifndef TCA_MANAGER_NODE_HPP_
#define TCA_MANAGER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tca9548a/tca9548a.hpp"
#include "tca9548a/srv/select_channel.hpp"
#include <memory>

/**
 * @class TcaManagerNode
 * @brief A ROS 2 node to manage a TCA9548A I2C multiplexer.
 * * This node provides a service to select one of the 8 channels on the TCA9548A,
 * allowing other nodes to communicate with specific I2C devices connected
 * through the multiplexer.
 */
class TcaManagerNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Tca Manager Node object.
   */
  TcaManagerNode();

private:
  /**
   * @brief Callback function to handle requests to the /select_channel service.
   * @param request The service request, containing the desired channel.
   * @param response The service response, indicating success or failure.
   */
  void handle_select_channel(
    const std::shared_ptr<tca9548a::srv::SelectChannel::Request> request,
    std::shared_ptr<tca9548a::srv::SelectChannel::Response> response);

  // ROS 2 service server for channel selection
  rclcpp::Service<tca9548a::srv::SelectChannel>::SharedPtr service_;
  
  // Pointer to the TCA9548A driver instance
  std::unique_ptr<Tca9548a> multiplexer_;
  
  // Stores the currently active channel to avoid redundant I2C writes
  int8_t current_channel_;
};

#endif // TCA_MANAGER_NODE_HPP_