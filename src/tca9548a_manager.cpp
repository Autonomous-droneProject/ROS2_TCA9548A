#include "tca9548a/tca9548a_manager.hpp"

namespace tca9548a {
Tca9548aManager::Tca9548aManager(const rclcpp::NodeOptions &options)
    : rclcpp::Node("tca_manager", options) {
  RCLCPP_INFO(this->get_logger(), "Starting TCA ros2 manager node...");

  // --- 1. Declare and get parameters ---
  i2c_bus_param_ =
      this->declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
  std::vector<int64_t> addresses_int64 =
      this->declare_parameter<std::vector<int64_t>>("tca_addresses",
                                                    std::vector<int64_t>{0x70});
  for (int64_t address_64 : addresses_int64) {
    tca_addresses_param_.push_back(static_cast<int>(address_64));
  }

  // --- 2. Initialize a driver for each TCA address ---
  RCLCPP_INFO(this->get_logger(), "Configured with I2C with bus: %s",
              i2c_bus_param_.c_str());
  for (int address_int : tca_addresses_param_) {
    uint8_t address = static_cast<uint8_t>(address_int);
    Tca9548a driver(i2c_bus_param_, address);
    if (!driver.open_bus()) {
      RCLCPP_FATAL(
          this->get_logger(),
          "Failed to open I2C bus for TCA at 0x%02X. Node will not function.",
          address);
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Opened I2C bus for TCA at 0x%02X.",
                  address);
      tca_drivers_.emplace(address, driver);
    }
  }

  // --- 3. Create channel server ---
  select_channel_service_ = this->create_service<tca9548a::srv::SelectChannel>(
      "select_channel",
      std::bind(&Tca9548aManager::handle_select_channel, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Service server for 'select_channel' is ready.");
  
}

void Tca9548aManager::handle_select_channel(
  const std::shared_ptr<tca9548a::srv::SelectChannel::Request> request,
  std::shared_ptr<tca9548a::srv::SelectChannel::Response> response) {
    // Check if TCA drivers exist in map
    // Check if the requested TCA address exists in our map
    if (tca_drivers_.find(request->address) == tca_drivers_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Received request for unknown TCA address: 0x%02X", request->address);
        response->success = false;
        return;
    }

    Tca9548a& driver = tca_drivers_.at(request->address);
        RCLCPP_INFO(this->get_logger(), "Received request to select channel %d on TCA 0x%02X", request->channel, request->address);
    bool success = driver.select_channel(request->channel);
    response->success = success;
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Successfully selected channel %d on TCA 0x%02X.", request->channel, request->address);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to select channel %d on TCA 0x%02X.", request->channel, request->address);
    }
  }
} // namespace tca9548a

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tca9548a::Tca9548aManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}