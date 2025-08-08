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

  virtual bool initialize(Tca9548a&driver, uint8_t channel) = 0;
  virtual bool configure(Tca9548a& driver, uint8_t channel) = 0;
  virtual ReadResult read(Tca9548a& driver, uint8_t channel) = 0;


};
}; // namespace tca9548a

#endif