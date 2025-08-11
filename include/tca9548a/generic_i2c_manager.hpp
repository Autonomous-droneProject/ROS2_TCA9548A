#ifndef TCA9548A_GENERIC_I2C_MANAGER_
#define TCA9548A_GENERIC_I2C_MANAGER_

#include "rclcpp/rclcpp.hpp"
#include "tca9548a/srv/config_device.hpp"
#include "tca9548a/srv/init_device.hpp"
#include "tca9548a/srv/read_device.hpp"
#include "tca9548a/srv/register_device.hpp"
#include "tca9548a/msg/sensor_data.hpp"
#include <map>
#include <string>

namespace tca9548a {
struct SensorConfig {
  uint8_t tca_address;
  uint8_t channel;
  uint8_t sensor_address;
  std::string topic_name;
  std::string device_type;
};

class GenericI2CManager : public rclcpp::Node {
public:
  explicit GenericI2CManager(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  rclcpp::Client<tca9548a::srv::ReadDevice>::SharedPtr read_device_client_;
  rclcpp::Client<tca9548a::srv::RegisterDevice>::SharedPtr
      register_device_client_;
  rclcpp::Client<tca9548a::srv::InitDevice>::SharedPtr init_device_client_;
  rclcpp::Client<tca9548a::srv::ConfigDevice>::SharedPtr config_device_client_;

  std::map<std::string, rclcpp::Publisher<tca9548a::msg::SensorData>::SharedPtr>
      publishers_;
  std::map<std::string, SensorConfig> sensor_configs_;

  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback();
};

}; // namespace tca9548a

#endif