#include "pca9685_hardware_interface/pca9685_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <pca9685_hardware_interface/pca9685_comm.hpp>


namespace pca9685_hardware_interface
{
hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Call parent class on_init
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read "i2c_device" parameter
  // If not found, use default "/dev/i2c-1"
  std::string i2c_device = "/dev/i2c-1";
  try {
    i2c_device = info_.hardware_parameters.at("i2c_device");
    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
    "got parameter I2C device: %s", i2c_device.c_str());
  } catch (const std::out_of_range & e) {
    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "Missing parameter: i2c_device. Using default: %s", i2c_device.c_str());
  }

  // Read "i2c_address" parameter
  // If not found, use default 0x40
  int i2c_address = 0x40;
  try {
    i2c_address = stoi(info_.hardware_parameters.at("i2c_address"));
    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "got parameter I2C address: %#04x", i2c_address);
  } catch (const std::out_of_range & e) {
    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "Missing parameter: i2c_address. Using default: %#04x", i2c_address);
  }

  // Read "pwm_frequency" parameter
  // Acceptable range: [24, 1526] Hz
  // If not found, use default 50.0
  double pwm_frequency = 50.0;
  try {
    pwm_frequency = stod(info_.hardware_parameters.at("pwm_frequency"));
    if (pwm_frequency < 24.0 || pwm_frequency > 1526.0) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "pwm_frequency out of range [24, 1526]: %f", pwm_frequency);
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "got parameter pwm_frequency: %f", pwm_frequency);
  } catch (const std::out_of_range & e) {
    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "Missing parameter: pwm_frequency. Using default: %f", pwm_frequency);
  }

  // TODO(rosblox) it would be nice to know if the connection fails
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
    "Initializing I2C connection to PCA9685 on bus %s and address %#04x",
    i2c_device.c_str(), i2c_address);
  pca = PiPCA9685::PCA9685(i2c_device, i2c_address);
  pca.set_pwm_freq(pwm_frequency);

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_  .resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // Pca9685System has one command interface on each output
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Read parameter channel
    // Acceptable range: [0, 15]
    // No duplicates
    // No default value. This parameter is mandatory.
    try {
      int channel = stoi(joint.parameters.at("channel"));
      if (channel < 0 || channel > 15) {
        RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"),
          "Joint %s parameter channel out of range [0, 15]: %d",
          joint.name.c_str(), channel);
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (std::find(channels_.begin(), channels_.end(), channel) != channels_.end()) {
        RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"),
          "Joint %s parameter channel conflict found: duplicate on channel %d",
          joint.name.c_str(), channel);
        return hardware_interface::CallbackReturn::ERROR;
      }
      channels_.emplace_back(channel);
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s got parameter channel: %d", joint.name.c_str(), channel);
    } catch (const std::out_of_range & e) {
      RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s missing parameter 'channel'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check that command interface is either velocity or position.
    // If velocity, treat this as a continuous rotation servo
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY &&
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %s command interface. '%s' or '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("Pca9685SystemHardware"),
      "Joint '%s' has %s command interface.", joint.name.c_str(),
      joint.command_interfaces[0].name.c_str());

    continuous_.emplace_back(
      joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY);

    // Only zero or one state interface allowed, and it must match the command interface
    if (joint.state_interfaces.size() > 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
          "Joint '%s' has %zu state interfaces found. 0 or 1 expected.",
          joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    } else if (joint.state_interfaces.size() == 1 &&
      joint.state_interfaces[0].name != joint.command_interfaces[0].name)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), joint.command_interfaces[0].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    } else {
      fake_states_.emplace_back(joint.state_interfaces.size() == 1);
    }

    // Read "pwm_low" parameter
    // Acceptable range: [0, 1000/pwm_frequency]
    // If not found, use default 1.0
    try {
      pwm_lows_.emplace_back(std::stod(joint.parameters.at("pwm_low")));
      if (pwm_lows_.back() < 0 || pwm_lows_.back() > 1000 / pwm_frequency) {
        RCLCPP_FATAL(
          rclcpp::get_logger("Pca9685SystemHardware"),
            "Joint '%s' parameter pwm_low is not reasonable. Units are ms, in the ballpark of"
            "1ms-2ms for standard 50Hz servos. Also check your frequency. pwm_low: %f",
            joint.name.c_str(), pwm_lows_.back());
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s got parameter pwm_low: %f", joint.name.c_str(), pwm_lows_.back());
    } catch (const std::out_of_range & e) {
      pwm_lows_.emplace_back(1.0);
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s missing parameter 'pwm_low', assuming default 1.0", joint.name.c_str());
    }

    // Read "pwm_high" parameter
    // Acceptable range: [0, 1000/pwm_frequency]
    // If not found, use default 2.0
    try {
      pwm_highs_.emplace_back(std::stod(joint.parameters.at("pwm_high")));
      if (pwm_highs_.back() < 0 || pwm_highs_.back() > 1000 / pwm_frequency) {
        RCLCPP_FATAL(
          rclcpp::get_logger("Pca9685SystemHardware"),
            "Joint '%s' parameter pwm_high is not reasonable. Units are ms, in the ballpark of"
            "1ms-2ms for standard 50Hz servos. Also check your frequency. pwm_high: %f",
            joint.name.c_str(), pwm_highs_.back());
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s got parameter pwm_high: %f", joint.name.c_str(), pwm_highs_.back());
    } catch (const std::out_of_range & e) {
      pwm_highs_.emplace_back(2.0);
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s missing parameter 'pwm_high', assuming default 2.0", joint.name.c_str());
    }

    // Read "pwm_zero" parameter
    // Acceptable range: [0, 1000/pwm_frequency]
    // If not found, use default (pwm_low + pwm_high)/2
    try {
      pwm_zeros_.emplace_back(std::stod(joint.parameters.at("pwm_zero")));
      if (pwm_zeros_.back() < 0 || pwm_zeros_.back() > 1000 / pwm_frequency) {
        RCLCPP_FATAL(
          rclcpp::get_logger("Pca9685SystemHardware"),
            "Joint '%s' parameter pwm_zero is not reasonable. Units are ms, in the ballpark of"
            "1ms-2ms for standard 50Hz servos. Also check your frequency. pwm_zero: %f",
            joint.name.c_str(), pwm_zeros_.back());
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "Joint %s got parameter pwm_zero: %f", joint.name.c_str(), pwm_zeros_.back());
    } catch (const std::out_of_range & e) {
      pwm_zeros_.emplace_back((pwm_lows_.back() + pwm_highs_.back()) / 2.0);
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s missing parameter 'pwm_zero', assuming default of (pwm_low + pwm_high)/2",
        joint.name.c_str());
    }

    // Read "scale" parameter
    // If not found, use default 1.0
    try {
      scales_.emplace_back(std::stod(joint.parameters.at("scale")));
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s got parameter scale: %f", joint.name.c_str(), scales_.back());
    } catch (const std::out_of_range & e) {
      scales_.emplace_back(1.0);
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s missing parameter 'scale', assuming default 1.0", joint.name.c_str());
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto i = 0u; i < info_.joints.size(); i++) {
    // Only export if this joint has a state interface specified
    if (fake_states_[i]) {
      // Match the state interface type to the command interface
      if (continuous_[i]) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_[i]));
      } else {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pca9685SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    if (continuous_[i]) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    } else {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < hw_commands_.size(); i++) {
    if (std::isnan(hw_commands_[i])) {
      if(continuous_[i]) {
        hw_commands_[i] = pulse_width_to_command(i, pwm_zeros_[i]);
        hw_states_[i] = hw_commands_[i];
      } else {
        hw_commands_[i] = 0.0;
        hw_states_[i] = 0.0;
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop all continuous joints
  // TODO(mjforan) unable to test this because of a bug in ros2_control
  // https://github.com/ros-controls/ros2_control/issues/2012
  // Do we need to manually run write() after setting command?
  for (auto i = 0u; i < hw_commands_.size(); i++) {
    if(continuous_[i]) {
      hw_commands_[i] = pulse_width_to_command(i, pwm_zeros_[i]);
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pca9685SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

double Pca9685SystemHardware::command_to_pulse_width(int joint, double command)
{
  double slope = (pwm_highs_[joint] - pwm_zeros_[joint]) / scales_[joint];
  double min_command = (pwm_lows_[joint] - pwm_zeros_[joint]) / slope;
  double clamped_command;
  if (scales_[joint] < 0) {
    clamped_command = std::clamp(command, scales_[joint], min_command);
  } else {
    clamped_command = std::clamp(command, min_command, scales_[joint]);
  }

  if (abs(clamped_command - command) > 0.000001) {
    RCLCPP_WARN(rclcpp::get_logger("Pca9685SystemHardware"),
      "Clamping command: %f -> %f", command, clamped_command);
  }

  hw_states_[joint] = clamped_command;

  return slope * clamped_command + pwm_zeros_[joint];
}

double Pca9685SystemHardware::pulse_width_to_command(int joint, double pulse_ms)
{
  double slope = (pwm_highs_[joint] - pwm_zeros_[joint]) / scales_[joint];
  return (pulse_ms - pwm_zeros_[joint]) / slope;
}

hardware_interface::return_type Pca9685SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (auto i = 0u; i < hw_commands_.size(); i++) {
    double pulse_width = command_to_pulse_width(i, hw_commands_[i]);

    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "Joint '%d' has command '%f', pulse_width: '%f'.", i, hw_commands_[i], pulse_width);

    pca.set_pwm_ms(channels_[i], pulse_width);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pca9685_hardware_interface::Pca9685SystemHardware, hardware_interface::SystemInterface)
