#include "tca9548a/generic_i2c_manager.hpp"
#include <limits>

namespace tca9548a {
GenericI2CManager::GenericI2CManager(const rclcpp::NodeOptions &options)
    : rclcpp::Node("vl53l1x_manager", options) {
  std::string tca_manager_node_name = this->declare_parameter<std::string>(
      "tca_manager_node_name", "tca9548a_manager");
  std::string read_service_name = "/" + tca_manager_node_name + "/read_device";
  std::string register_service_name =
      "/" + tca_manager_node_name + "/register_device";
  std::string init_service_name = "/" + tca_manager_node_name + "/init_device";
  std::string config_service_name =
      "/" + tca_manager_node_name + "/config_device";
  double publish_rate_param =
      this->declare_parameter<double>("publish_rate", 10.0);

  // Create clients for each service
  read_device_client_ =
      this->create_client<tca9548a::srv::ReadDevice>(read_service_name);
  register_device_client_ =
      this->create_client<tca9548a::srv::RegisterDevice>(register_service_name);
  init_device_client_ =
      this->create_client<tca9548a::srv::InitDevice>(init_service_name);
  config_device_client_ =
      this->create_client<tca9548a::srv::ConfigDevice>(config_service_name);

  // Wait for all clients to be available before proceeding
  if (!read_device_client_->wait_for_service(std::chrono::seconds(5)) ||
      !register_device_client_->wait_for_service(std::chrono::seconds(5)) ||
      !init_device_client_->wait_for_service(std::chrono::seconds(5)) ||
      !config_device_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_FATAL(this->get_logger(), "Not all services from the TCA manager "
                                     "are available. Node cannot start.");
    return;
  }

  auto sensor_list =
      this->declare_parameter<std::vector<std::string>>("sensor_list", std::vector<std::string>{});

  for (const auto &sensor_name : sensor_list) {
    // Get parameters for each individual sensor from the YAML file
    std::string prefix = sensor_name + ".";

    SensorConfig config;
    config.tca_address = static_cast<uint8_t>(
        this->declare_parameter<int>(prefix + "tca_address", 0x70));
    config.channel = static_cast<uint8_t>(
        this->declare_parameter<int>(prefix + "channel", 0));
    config.sensor_address = static_cast<uint8_t>(
        this->declare_parameter<int>(prefix + "sensor_address", 0x29));
    config.topic_name = this->declare_parameter<std::string>(
        prefix + "topic_name", sensor_name);
    config.device_type = this->declare_parameter<std::string>(
        prefix + "device_type", sensor_name);

    sensor_configs_[sensor_name] = config;
    publishers_[sensor_name] =
        this->create_publisher<sensor_msgs::msg::Range>(config.topic_name, 10);

    // Register the device
    auto register_req =
        std::make_shared<tca9548a::srv::RegisterDevice::Request>();
    register_req->tca_address = config.tca_address;
    register_req->channel = config.channel;
    register_req->device_type = config.device_type;
    auto register_future =
        register_device_client_->async_send_request(register_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           register_future) !=
            rclcpp::FutureReturnCode::SUCCESS ||
        !register_future.get()->success) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to register sensor '%s'. Skipping.",
                   sensor_name.c_str());
      continue;
    }

    // Initialize the device
    auto init_req = std::make_shared<tca9548a::srv::InitDevice::Request>();
    init_req->tca_address = config.tca_address;
    init_req->channel = config.channel;
    auto init_future = init_device_client_->async_send_request(init_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           init_future) !=
            rclcpp::FutureReturnCode::SUCCESS ||
        !init_future.get()->success) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to initialize sensor '%s'. Skipping.",
                   sensor_name.c_str());
      continue;
    }

    // Configure the device
    auto config_req = std::make_shared<tca9548a::srv::ConfigDevice::Request>();
    config_req->tca_address = config.tca_address;
    config_req->channel = config.channel;
    auto config_future = config_device_client_->async_send_request(config_req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           config_future) !=
            rclcpp::FutureReturnCode::SUCCESS ||
        !config_future.get()->success) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to configure sensor '%s'. Skipping.",
                   sensor_name.c_str());
      continue;
    }
  }

  auto publish_period =
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_param));
  timer_ = this->create_wall_timer(
      publish_period, std::bind(&GenericI2CManager::timer_callback, this));

  RCLCPP_INFO(this->get_logger(),
              "Manager initialized with %zu sensors. Publishing at %.2f Hz.",
              sensor_list.size(), publish_rate_param);
}

void GenericI2CManager::timer_callback() {
  for (const auto &pair : sensor_configs_) {
    const auto &sensor_name = pair.first;
    const auto &config = pair.second;

    auto request = std::make_shared<tca9548a::srv::ReadDevice::Request>();
    request->tca_address = config.tca_address;
    request->channel = config.channel;

    auto client = read_device_client_;

    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service for TCA 0x%02X not available.",
                   config.tca_address);
      continue;
    }

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result_future) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service for sensor '%s'",
                   sensor_name.c_str());
      continue;
    }

    auto result = result_future.get();
    auto message = sensor_msgs::msg::Range();

    if (result->success) {
      message.header.stamp = this->now();
      message.header.frame_id = sensor_name;
      message.range = result->reading / 1000.0f;
      // ... (fill in other Range fields)
      publishers_.at(sensor_name)->publish(message);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to read sensor '%s'. Publishing invalid data.",
                  sensor_name.c_str());
      message.range = std::numeric_limits<float>::infinity();
      publishers_.at(sensor_name)->publish(message);
    }
  }
}
} // namespace tca9548a

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tca9548a::GenericI2CManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

