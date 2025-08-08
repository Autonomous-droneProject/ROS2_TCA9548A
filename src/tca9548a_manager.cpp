#include "tca9548a/tca9548a_manager.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
namespace tca9548a {
Tca9548aManager::Tca9548aManager(const rclcpp::NodeOptions &options)
    : rclcpp::Node("tca_manager", options), device_loader_("tca9548a", "tca9548a::I2CDevice") {
  RCLCPP_INFO(this->get_logger(), "Starting TCA ros2 manager node...");

  // --- 1. Declare and get parameters ---
  i2c_bus_param_ =
      this->declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
  std::vector<int64_t> addresses_int64 =
      this->declare_parameter<std::vector<int64_t>>("tca_addresses",
                                                    std::vector<int64_t>{0x70});
  for (int64_t address_64 : addresses_int64) {
    tca_addresses_param_.push_back(static_cast<uint8_t>(address_64));
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

  // --- 3a. Initialize registration service ---
  register_device_service_ =
      this->create_service<tca9548a::srv::RegisterDevice>(
          "register_device",
          std::bind(&Tca9548aManager::handle_register_device, this, _1, _2));

  // --- 3b. Initialize init service ---
  init_sensor_service_ = this->create_service<tca9548a::srv::InitDevice>(
      "init_device",
      std::bind(&Tca9548aManager::init_sensor_device, this, _1, _2));

  // --- 3c. Initialize config service ---
  config_sensor_service_ = this->create_service<tca9548a::srv::ConfigDevice>(
      "config_device",
      std::bind(&Tca9548aManager::config_sensor_device, this, _1, _2));

  // --- 3d. Initialize read service
  read_sensor_service_ = this->create_service<tca9548a::srv::ReadDevice>(
      "read_device",
      std::bind(&Tca9548aManager::read_sensor_device, this, _1, _2));

  RCLCPP_INFO(this->get_logger(),
              "Service server for 'select_channel' is ready.");
}

/**
 * @brief registers the i2c device. Does not init the device
 */
void Tca9548aManager::handle_register_device(
    const std::shared_ptr<tca9548a::srv::RegisterDevice::Request> request,
    std::shared_ptr<tca9548a::srv::RegisterDevice::Response> response) {

  // --- 1. Acquire the mutex to protect the shared resource ---
  std::lock_guard<std::mutex> lock(tca_mutex_);

  RCLCPP_INFO(this->get_logger(),
              "Attempting to register device '%s' on TCA 0x%02X, channel %d.",
              request->device_type.c_str(), request->tca_address,
              request->channel);

  // --- 2. Input Validation ---
  if (tca_drivers_.find(request->tca_address) == tca_drivers_.end()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Request failed: Unknown TCA address 0x%02X.",
                 request->tca_address);
    response->success = false;
    return;
  }

  auto &device_map_for_tca = devices_.at(request->tca_address);
  if (device_map_for_tca.find(request->channel) != device_map_for_tca.end()) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Request failed: Channel %d on TCA 0x%02X is already occupied.",
        request->channel, request->tca_address);
    response->success = false;
    return;
  }

  // --- 3. Create the Device using a Factory Pattern ---
  pluginlib::UniquePtr<I2CDevice> new_device;
  try {
    new_device = create_device(request->device_type);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create device of type '%s': %s",
                 request->device_type.c_str(), e.what());
    response->success = false;
    return;
  }

  if (!new_device) {
    RCLCPP_ERROR(this->get_logger(),
                 "Request failed: Unknown device type '%s'.",
                 request->device_type.c_str());
    response->success = false;
    return;
  }

  /*
    // --- 4. Initialize the device ---
  Tca9548a &tca_driver = tca_drivers_.at(request->tca_address);
  if (!new_device->initialize(tca_driver, request->channel)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Initialization of device '%s' on channel %d failed.",
                 request->device_type.c_str(), request->channel);
    response->success = false;
    return;
  }
  */

  // --- 5. Add the device to the map and return success ---
  devices_.at(request->tca_address)
      .emplace(request->channel, std::move(new_device));
  response->success = true;
  RCLCPP_INFO(this->get_logger(),
              "Successfully registered device '%s' on TCA "
              "0x%02X, channel %d.",
              request->device_type.c_str(), request->tca_address,
              request->channel);
}

pluginlib::UniquePtr<I2CDevice>
Tca9548aManager::create_device(const std::string &device_type) {
  try {
    return device_loader_.createUniqueInstance(device_type);
  } catch (const pluginlib::PluginlibException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create device of type '%s': %s",
                 device_type.c_str(), ex.what());
    return nullptr;
  }
}

/**
 * @brief preform init of a generic i2c device.
 */
void Tca9548aManager::init_sensor_device(const std::shared_ptr<tca9548a::srv::InitDevice::Request> request, std::shared_ptr<tca9548a::srv::InitDevice::Response> response) {
  // --- 1. Acquire the mutex to protect the shared resource ---
  std::lock_guard<std::mutex> lock(tca_mutex_);

  RCLCPP_INFO(this->get_logger(), "Attempting to initialize device on TCA 0x%02X, channel %d.",
              request->tca_address, request->channel);
  
  // --- 2. Input Validation ---
  if (tca_drivers_.find(request->tca_address) == tca_drivers_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Request failed: Unknown TCA address 0x%02X.", request->tca_address);
    response->success = false;
    return;
  }
  
  auto& device_map_for_tca = devices_.at(request->tca_address);
  if (device_map_for_tca.find(request->channel) == device_map_for_tca.end()) {
    RCLCPP_ERROR(this->get_logger(), "Request failed: No device is registered on channel %d.", request->channel);
    response->success = false;
    return;
  }

  // --- 3. Perform the polymorphic call to initialize the device ---
  Tca9548a& tca_driver = tca_drivers_.at(request->tca_address);
  I2CDevice& device = *devices_.at(request->tca_address).at(request->channel);
  tca_driver.open_bus();
  tca_driver.select_channel(request->channel);
  bool success = device.initialize();
  tca_driver.close_bus();

  // --- 4. Return the result ---
  response->success = success;
  if (success) {
    RCLCPP_INFO(this->get_logger(), "Successfully initialized device on TCA 0x%02X, channel %d.",
                request->tca_address, request->channel);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize device on TCA 0x%02X, channel %d.",
                 request->tca_address, request->channel);
  }
}

/**
 * @brief preform config of a generic i2c device.
 */
void Tca9548aManager::config_sensor_device(
    const std::shared_ptr<tca9548a::srv::ConfigDevice::Request> request,
    std::shared_ptr<tca9548a::srv::ConfigDevice::Response> response) {
  
  std::lock_guard<std::mutex> lock(tca_mutex_);

  RCLCPP_INFO(this->get_logger(), "Attempting to configure device on TCA 0x%02X, channel %d.",
              request->tca_address, request->channel);
  
  if (tca_drivers_.find(request->tca_address) == tca_drivers_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Request failed: Unknown TCA address 0x%02X.", request->tca_address);
    response->success = false;
    return;
  }
  
  auto& device_map_for_tca = devices_.at(request->tca_address);
  if (device_map_for_tca.find(request->channel) == device_map_for_tca.end()) {
    RCLCPP_ERROR(this->get_logger(), "Request failed: No device is registered on channel %d.", request->channel);
    response->success = false;
    return;
  }

  Tca9548a& tca_driver = tca_drivers_.at(request->tca_address);
  I2CDevice& device = *devices_.at(request->tca_address).at(request->channel);
  tca_driver.open_bus();
  tca_driver.select_channel(request->channel);
  bool success = device.configure();
  tca_driver.close_bus();

  response->success = success;
  if (success) {
    RCLCPP_INFO(this->get_logger(), "Successfully configured device on TCA 0x%02X, channel %d.",
                request->tca_address, request->channel);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure device on TCA 0x%02X, channel %d.",
                 request->tca_address, request->channel);
  }
}

/**
 * @brief preform read of a generic i2c device.
 */
void Tca9548aManager::read_sensor_device(
    const std::shared_ptr<tca9548a::srv::ReadDevice::Request> request,
    std::shared_ptr<tca9548a::srv::ReadDevice::Response> response) {
  
  std::lock_guard<std::mutex> lock(tca_mutex_);

  RCLCPP_INFO(this->get_logger(), "Attempting to read device on TCA 0x%02X, channel %d.",
              request->tca_address, request->channel);
  
  if (tca_drivers_.find(request->tca_address) == tca_drivers_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Request failed: Unknown TCA address 0x%02X.", request->tca_address);
    response->success = false;
    return;
  }
  
  auto& device_map_for_tca = devices_.at(request->tca_address);
  if (device_map_for_tca.find(request->channel) == device_map_for_tca.end()) {
    RCLCPP_ERROR(this->get_logger(), "Request failed: No device is registered on channel %d.", request->channel);
    response->success = false;
    return;
  }

  Tca9548a& tca_driver = tca_drivers_.at(request->tca_address);
  I2CDevice& device = *devices_.at(request->tca_address).at(request->channel);
  tca_driver.open_bus();
  tca_driver.select_channel(request->channel);
  ReadResult result = device.read();
  tca_driver.close_bus();

  response->success = result.success;
  if (result.success) {
    response->reading = result.reading;
    RCLCPP_INFO(this->get_logger(), "Successfully read from device on TCA 0x%02X, channel %d. Distance: %.2fmm",
                request->tca_address, request->channel, result.reading);
  } else {
    response->reading = 0.0f;
    RCLCPP_ERROR(this->get_logger(), "Failed to read from device on TCA 0x%02X, channel %d.",
                 request->tca_address, request->channel);
  }
}
} // namespace tca9548a

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tca9548a::Tca9548aManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}