#include "tca9548a/tca9548a.hpp" // Adjusted include path

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

namespace tca9548a {
Tca9548a::Tca9548a(const std::string &i2c_bus, uint8_t address)
    : i2c_bus_(i2c_bus), address_(address), fd_(-1) {}

Tca9548a::~Tca9548a() {
  if (fd_ != -1) {
    close_bus();
  }
}

bool Tca9548a::open_bus() {
  fd_ = open(i2c_bus_.c_str(), O_RDWR);
  if (fd_ < 0) {
    return false;
  }
  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    return false;
  }
  return true;
}

void Tca9548a::close_bus() {
  close(fd_);
  fd_ = -1;
}

bool Tca9548a::select_channel(uint8_t channel) {
  if (channel > 7) {
    return false;
  }
  uint8_t buffer = 1 << channel;
  if (write(fd_, &buffer, 1) != 1) {
    return false;
  }
  return true;
}
} // namespace tca9548a
