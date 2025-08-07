#ifndef TCA9548A_HPP_
#define TCA9548A_HPP_

#include <cstdint>
#include <string>

namespace tca9548a {
class Tca9548a {
public:
  Tca9548a(const std::string &i2c_bus = "/dev/i2c-1", uint8_t address = 0x70);
  ~Tca9548a();
  bool open_bus();
  void close_bus();
  bool select_channel(uint8_t channel);

private:
  std::string i2c_bus_;
  uint8_t address_;
  int fd_; // File descriptor for the i2c bus
};
} // namespace tca9548a

#endif