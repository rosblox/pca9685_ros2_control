#ifndef PCA9685_HARDWARE_INTERFACE__I2CPERIPHERAL_HPP_
#define PCA9685_HARDWARE_INTERFACE__I2CPERIPHERAL_HPP_

#include <cstdint>
#include <string>

namespace PiPCA9685
{

class I2CPeripheral {
public:
  I2CPeripheral(const std::string & device, const uint8_t address);
  ~I2CPeripheral();

  void WriteRegisterByte(const uint8_t register_address, const uint8_t value);

  uint8_t ReadRegisterByte(const uint8_t register_address);

private:
  int bus_fd;

  void OpenBus(const std::string & device);
  void ConnectToPeripheral(const uint8_t address);
};

}  // namespace PiPCA9685

#endif  // PCA9685_HARDWARE_INTERFACE__I2CPERIPHERAL_HPP_
