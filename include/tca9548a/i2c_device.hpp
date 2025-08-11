#ifndef TCA9548A_I2C_DEVICE_HPP
#define TCA9548A_I2C_DEVICE_HPP

#include <cstdint>
#include <vector>
#include "tca9548a/msg/sensor_data.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tca9548a {
class Tca9548a; // Forward declaration
class I2CDevice {
public:
  virtual ~I2CDevice() = default;

  /**
   * @brief init function called when init service is called
   */
  virtual bool initialize() = 0;
  /**
   * @brief config function called when config service is called
   */
  virtual bool configure() = 0;
  /**
   * @brief read function called when read function is called
   * @return tca9548a::msg::SensorData must be populated in function
   */
  virtual tca9548a::msg::SensorData read() {
    tca9548a::msg::SensorData message;

    // PSEUDOCODE for reading data
    float distance_mm = 123.45f;
    bool read_success = true;

    if (read_success) {
        message.header.stamp = rclcpp::Clock().now();
        message.device_name = "vl53l1x_sensor";
        
        diagnostic_msgs::msg::KeyValue distance_kv;
        distance_kv.key = "distance_mm";
        distance_kv.value = std::to_string(distance_mm);
        message.values.push_back(distance_kv);
    } else {
        // Return an empty message with a null timestamp to signal failure
        message.header.stamp = rclcpp::Time(0, 0);
    }
    return message;
  }
};
}; // namespace tca9548a

#endif