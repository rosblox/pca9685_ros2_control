#ifndef PCA9685_HARDWARE_INTERFACE__PCA9685_COMM_HPP_
#define PCA9685_HARDWARE_INTERFACE__PCA9685_COMM_HPP_

#include <string>
#include <memory>

namespace PiPCA9685
{

class I2CPeripheral;

class PCA9685 {
public:
  explicit PCA9685(const std::string & device = "/dev/i2c-1", int address = 0x40);
  ~PCA9685();

  PCA9685 & operator=(PCA9685 &&) noexcept;

  void set_pwm_freq(const double freq_hz);

  void set_pwm(const int channel, const uint16_t on, const uint16_t off);

  void set_all_pwm(const uint16_t on, const uint16_t off);

  void set_pwm_ms(const int channel, const double ms);

private:
  std::unique_ptr<I2CPeripheral> i2c_dev;

  // Default frequency pulled from PCA9685 datasheet.
  double frequency = 200.0;
};

}  // namespace PiPCA9685

#endif  // PCA9685_HARDWARE_INTERFACE__PCA9685_COMM_HPP_
