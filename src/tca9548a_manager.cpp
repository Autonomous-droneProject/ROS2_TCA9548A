#include "tca9548a/tca9548a_manager.hpp"

namespace tca9548a {

Tca9548aManager::Tca9548aManager(const rclcpp::NodeOptions &options)
    : rclcpp::Node("tca9548a_manager", options) {
  RCLCPP_INFO(this->get_logger(), "Tca9548aManager node starting...");

  // Declare a parameter for the list of I2C addresses to manage
  // Get the parameter as a vector of long int
  std::vector<long int> addresses_long =
      this->declare_parameter<std::vector<long int>>(
          "i2c_addresses", std::vector<long int>{112, 113});

  // Create a new vector for the int addresses with the same size
  std::vector<int> addresses_int(addresses_long.size());

  // Use std::transform to convert the vector
  std::transform(
      addresses_long.begin(), addresses_long.end(), addresses_int.begin(),
      [](long int address_long) { return static_cast<int>(address_long); });
  // Create a service client for each address in the list
  for (int address_int : addresses_int) {
    // Convert the integer address back to hex for the service name
    std::stringstream ss;
    ss << std::hex << address_int;
    std::string service_name = "/tca_node_0x" + ss.str() + "/select_channel";

    RCLCPP_INFO(this->get_logger(), "Creating client for service: %s",
                service_name.c_str());

    // Create and store the new client in the vector
    auto client =
        this->create_client<tca9548a::srv::SelectChannel>(service_name);
    clients_.push_back(client);
  }

  // Create a timer to periodically call the service
  // This timer will call the `timer_callback` function every 500ms
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&Tca9548aManager::timer_callback, this));
}

// Timer callback function: This is where the manager's logic resides.
void Tca9548aManager::timer_callback() {
  // In this example, we'll cycle through channels 0 to 7 on the first device
  // (client index 0).
  if (clients_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No clients to call.");
    return;
  }

  // Call the service with the current channel_counter_ value
  call_select_channel_service(0, static_cast<uint8_t>(channel_counter_));

  // Increment and wrap the counter around to 0 after channel 7
  channel_counter_ = (channel_counter_ + 1) % 8;
}

// Helper function to call a service
void Tca9548aManager::call_select_channel_service(size_t client_index,
                                                  uint8_t channel) {
  if (client_index >= clients_.size()) {
    RCLCPP_ERROR(this->get_logger(), "Client index out of bounds.");
    return;
  }

  auto client = clients_[client_index];
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Service not available for client %zu.",
                 client_index);
    return;
  }

  auto request = std::make_shared<tca9548a::srv::SelectChannel::Request>();
  request->channel = channel;

  // Call the service asynchronously
  auto result_future = client->async_send_request(request);

  // You can handle the response here if needed, or in a separate callback.
  // For this example, we'll just log that the request was sent.
  RCLCPP_INFO(this->get_logger(),
              "Service request sent to client %zu for channel %d.",
              client_index, channel);
}

} // namespace tca9548a

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tca9548a::Tca9548aManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}