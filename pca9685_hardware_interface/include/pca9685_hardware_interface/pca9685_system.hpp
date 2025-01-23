#ifndef PCA9685_HARDWARE_INTERFACE__PCA9685_SYSTEM_HPP_
#define PCA9685_HARDWARE_INTERFACE__PCA9685_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "pca9685_hardware_interface/visibility_control.hpp"
#include <pca9685_hardware_interface/pca9685_comm.hpp>

namespace pca9685_hardware_interface
{
class Pca9685SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Pca9685SystemHardware);

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  PCA9685_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<int> channels_;
  std::vector<bool> continuous_;
  std::vector<bool> fake_states_;
  std::vector<double> pwm_lows_;
  std::vector<double> pwm_highs_;
  std::vector<double> pwm_zeros_;
  std::vector<double> scales_;
  PiPCA9685::PCA9685 pca;
  double command_to_pulse_width(int joint, double command);
  double pulse_width_to_command(int joint, double pulse_ms);
};

}  // namespace pca9685_hardware_interface

#endif  // PCA9685_HARDWARE_INTERFACE__PCA9685_SYSTEM_HPP_
