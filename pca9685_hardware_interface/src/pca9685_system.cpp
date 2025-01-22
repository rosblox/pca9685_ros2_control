#include "pca9685_hardware_interface/pca9685_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <pca9685_hardware_interface/pca9685_comm.h>


namespace pca9685_hardware_interface
{
hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string i2c_device = "/dev/i2c-1";
  try{
    i2c_device = info_.hardware_parameters.at("i2c_device");
    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
    "got parameter I2C device: %s", i2c_device.c_str());
  }
  catch (const std::out_of_range& e){
    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "Missing parameter: i2c_device. Using default: %s", i2c_device.c_str());
  }

  int i2c_address = 0x40;
  try{
    i2c_address = stoi(info_.hardware_parameters.at("i2c_address"));
    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "got parameter I2C address: %#04x", i2c_address);
  }
  catch (const std::out_of_range& e){
    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "Missing parameter: i2c_address. Using default: %#04x", i2c_address);
  }

  double pwm_frequency = 50.0;
  try{
    pwm_frequency = stod(info_.hardware_parameters.at("pwm_frequency"));
    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "got parameter pwm_frequency: %f", pwm_frequency);
  }
  catch (const std::out_of_range& e){
    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "Missing parameter: pwm_frequency. Using default: %f", pwm_frequency);
  }

  // TODO it would be nice to know if the connection fails
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
    "Initializing I2C connection to PCA9685 on bus %s and address %#04x",
    i2c_device.c_str(), i2c_address);
  pca = PiPCA9685::PCA9685(i2c_device, i2c_address);
  pca.set_pwm_freq(pwm_frequency);

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Pca9685System has one command interface on each output
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    try {
      channels_.emplace_back(stoi(joint.parameters.at("channel")));
      if (channels_.back() < 0 || channels_.back() > 15) {
        RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"),
          "Joint %s parameter channel out of range [0, 15]: %d",
          joint.name.c_str(), channels_.back());
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s got parameter channel: %d", joint.name.c_str(), channels_.back());
    }
    catch (const std::out_of_range& e){
      RCLCPP_FATAL(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s missing parameter 'channel'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY &&
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %s command interface. '%s' or '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_POSITION,);
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("Pca9685SystemHardware"),
      "Joint '%s' has %s command interface.", joint.name.c_str(),
      joint.command_interfaces[0].name.c_str());

    continuous_.emplace_back(
      joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY);

    if (joint.state_interfaces.size() >= 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
          "Joint '%s' has %zu state interfaces found. 0 or 1 expected.",
          joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    else if (joint.state_interfaces.size() == 1 &&
    joint.state_interfaces[0].name != joint.command_interfaces[0].name)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), joint.command_interfaces[0].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    else {
      fake_states_.emplace_back(joint.state_interfaces.size() == 1);
    }

    try {
      pwm_lows_.emplace_back(std::stod(joint.parameters.at("pwm_low")));
      if (pwm_lows_.back() < 0 || pwm_lows_.back() > 1000/pwm_frequency) {
        RCLCPP_FATAL(
          rclcpp::get_logger("Pca9685SystemHardware"),
            "Joint '%s' parameter pwm_low is not reasonable. Units are ms, in the ballpark of"
            "1ms-2ms for standard 50Hz servos. Also check your frequency. pwm_low: %f",
            joint.name.c_str(), pwm_lows_.back());
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s got parameter pwm_low: %f", joint.name.c_str(), pwm_lows_.back());
    }
    catch (const std::out_of_range& e){
      pwm_lows_.emplace_back(1.0);
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s missing parameter 'pwm_low', assuming default 1.0", joint.name.c_str());
    }
    try {
      pwm_highs_.emplace_back(std::stod(joint.parameters.at("pwm_high")));
      if (pwm_highs_.back() < 0 || pwm_highs_.back() > 1000/pwm_frequency){
        RCLCPP_FATAL(
          rclcpp::get_logger("Pca9685SystemHardware"),
            "Joint '%s' parameter pwm_high is not reasonable. Units are ms, in the ballpark of"
            "1ms-2ms for standard 50Hz servos. Also check your frequency. pwm_high: %f",
            joint.name.c_str(), pwm_highs_.back());
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s got parameter pwm_high: %f", joint.name.c_str(), pwm_highs_.back());
    }
    catch (const std::out_of_range& e){
      pwm_highs_.emplace_back(2.0);
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s missing parameter 'pwm_high', assuming default 2.0", joint.name.c_str());
    }

    try {
      pwm_zeros_.emplace_back(std::stod(joint.parameters.at("pwm_zero")));
      if (pwm_zeros_.back() < 0 || pwm_zeros_.back() > 1000/pwm_frequency){
        RCLCPP_FATAL(
          rclcpp::get_logger("Pca9685SystemHardware"),
            "Joint '%s' parameter pwm_zero is not reasonable. Units are ms, in the ballpark of"
            "1ms-2ms for standard 50Hz servos. Also check your frequency. pwm_zero: %f",
            joint.name.c_str(), pwm_zeros_.back());
        return hardware_interface::CallbackReturn::ERROR;
      }
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "Joint %s got parameter pwm_zero: %f", joint.name.c_str(), pwm_zeros_.back());
      if (!continuous_.back()) {
        RCLCPP_WARN(rclcpp::get_logger("Pca9685SystemHardware"),
          "Position joint %s got parameter pwm_zero, but this only applies to velocity joints",
          joint.name.c_str());
      }
    }
    catch (const std::out_of_range& e){
      pwm_zeros_.emplace_back(1.5);
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s missing parameter 'pwm_zero', assuming default 1.5", joint.name.c_str());
    }

    try {
      vel_scales_.emplace_back(std::stod(joint.parameters.at("vel_scale")));
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s got parameter vel_scale: %f", joint.name.c_str(), vel_scales_.back());
      if (!continuous_.back()) {
        RCLCPP_WARN(rclcpp::get_logger("Pca9685SystemHardware"),
          "Position joint %s got parameter vel_scale, but this only applies to velocity joints",
          joint.name.c_str());
      }
    }
    catch (const std::out_of_range& e){
      vel_scales_.emplace_back(1.0);
      RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint %s missing parameter 'vel_scale', assuming default 1.0", joint.name.c_str());
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    if (fake_states_[i]) {
      if (continuous_[i]){
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
      }
      else {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pca9685SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    if (continuous_[i]) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
    else {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_commands_[i]))
    {
      if(continuous_[i])
        hw_commands_[i] = pwm_zeros_[i];
      else
        hw_commands_[i] = 0.0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pca9685SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  return hardware_interface::return_type::OK;
}

double Pca9685SystemHardware::command_to_pulse_width(int joint, double command){

    double clamped_command;
    double slope;
    double offset;
    if (continuous_[joint]){
      clamped_command = std::clamp(command, -1.0*vel_scales_[joint], vel_scales_[joint]);
      slope = (pwm_highs_[joint]-pwm_lows_[joint])/(vel_scales_[joint]*2.0);
      offset = pwm_zeros_[joint];
    }
    else {
      clamped_command = std::clamp(command, -1.0*M_PI, M_PI);
      slope = (pwm_highs_[joint]-pwm_lows_[joint])/(M_PI*2.0);
      offset = (pwm_highs_[joint]+pwm_lows_[joint])/2;
    }

    if (clamped_command != command)
      RCLCPP_WARN(rclcpp::get_logger("Pca9685SystemHardware"),
        "Clamping command: %f -> %f", command, clamped_command);

    return slope * clamped_command + offset;
}

hardware_interface::return_type Pca9685SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    double pulse_width = command_to_pulse_width(i, hw_commands_[i]);

    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685SystemHardware"),
      "Joint '%d' has command '%f', pulse_width: '%f'.", i, hw_commands_[i], pulse_width);

    pca.set_pwm_ms(i, pulse_width);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pca9685_hardware_interface::Pca9685SystemHardware, hardware_interface::SystemInterface)
