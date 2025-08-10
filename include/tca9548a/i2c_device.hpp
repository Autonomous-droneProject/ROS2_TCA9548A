#ifndef TCA9548A_I2C_DEVICE_HPP
#define TCA9548A_I2C_DEVICE_HPP

#include <cstdint>
#include <vector>

namespace tca9548a {
class Tca9548a; // Forward declaration
struct ReadResult {
    float reading;
    bool success;
};
class I2CDevice {
public:
  virtual ~I2CDevice() = default;

  virtual bool initialize() = 0;
  virtual bool configure() = 0;
  virtual ReadResult read() = 0;


};
}; // namespace tca9548a

#endif