#ifndef TCA9548A_TCA9548A_MANAGER_HPP_
#define TCA9548A_TCA9548A_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "tca9548a/i2c_device.hpp"
#include "tca9548a/srv/config_device.hpp"
#include "tca9548a/srv/init_device.hpp"
#include "tca9548a/srv/read_device.hpp"
#include "tca9548a/srv/register_device.hpp"
#include "tca9548a/tca9548a.hpp"
#include <functional>
#include <mutex>
#include <pluginlib/class_loader.hpp>

namespace tca9548a {

class Tca9548aManager : public rclcpp::Node {
public:
  explicit Tca9548aManager(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  std::map<uint8_t, Tca9548a> tca_drivers_;
  std::mutex tca_mutex_;
  std::map<uint8_t, std::map<uint8_t, pluginlib::UniquePtr<I2CDevice>>>
      devices_; // outermost: tca address, inner: channel #

  std::string i2c_bus_param_;
  std::vector<uint8_t> tca_addresses_param_;

  rclcpp::Service<tca9548a::srv::RegisterDevice>::SharedPtr
      register_device_service_;
  rclcpp::Service<tca9548a::srv::InitDevice>::SharedPtr init_sensor_service_;
  rclcpp::Service<tca9548a::srv::ConfigDevice>::SharedPtr
      config_sensor_service_;
  rclcpp::Service<tca9548a::srv::ReadDevice>::SharedPtr
      read_sensor_service_;

  void handle_register_device(
      const std::shared_ptr<tca9548a::srv::RegisterDevice::Request> request,
      std::shared_ptr<tca9548a::srv::RegisterDevice::Response> response);
  void init_sensor_device(
      const std::shared_ptr<tca9548a::srv::InitDevice::Request> request,
      std::shared_ptr<tca9548a::srv::InitDevice::Response> response);
  void config_sensor_device(
      const std::shared_ptr<tca9548a::srv::ConfigDevice::Request> request,
      std::shared_ptr<tca9548a::srv::ConfigDevice::Response> response);
  void read_sensor_device(
      const std::shared_ptr<tca9548a::srv::ReadDevice::Request> request,
      std::shared_ptr<tca9548a::srv::ReadDevice::Response> response);

  pluginlib::ClassLoader<tca9548a::I2CDevice> device_loader_;
  pluginlib::UniquePtr<I2CDevice> create_device(const std::string &device_type);
};

} // namespace tca9548a

#endif // TCA9548A_TCA9548A_MANAGER_HPP_