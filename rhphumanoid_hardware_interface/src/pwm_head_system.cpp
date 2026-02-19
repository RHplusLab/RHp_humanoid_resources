// Copyright 2025 RHp Humanoid Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rhphumanoid_hardware_interface/pwm_head_system.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rhphumanoid_hardware
{

static const char * kLoggerName = "PWMHeadSystemHardware";

// ---------------------------------------------------------------------------
// Helper: read a hardware parameter with a default value
// ---------------------------------------------------------------------------
static std::string get_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key,
  const std::string & default_value)
{
  auto it = params.find(key);
  return (it != params.end()) ? it->second : default_value;
}

// ---------------------------------------------------------------------------
// on_init
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn PWMHeadSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Validate joint count ---
  if (info_.joints.size() != 2) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kLoggerName),
      "Expected exactly 2 joints (pan, tilt), got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Validate each joint has 1 position command + 1 position state ---
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' has %zu command interfaces, expected 1",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' command interface is '%s', expected '%s'",
        joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' has %zu state interfaces, expected 1",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' state interface is '%s', expected '%s'",
        joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // --- Read hardware parameters ---
  const auto & p = info_.hardware_parameters;

  use_mock_ = (get_param(p, "use_mock", "true") == "true");

  // PWM chip / channel configuration
  chip_pan_ = get_param(p, "pwm_chip_pan", "pwmchip3");
  channel_pan_ = std::stoi(get_param(p, "pwm_channel_pan", "0"));
  chip_tilt_ = get_param(p, "pwm_chip_tilt", "pwmchip2");
  channel_tilt_ = std::stoi(get_param(p, "pwm_channel_tilt", "0"));

  // PWM period (shared for both servos)
  const int period_ns = std::stoi(get_param(p, "pwm_period_ns", "20000000"));

  // Per-servo duty cycle calibration
  // Pan:  0°=600k ns, 90°=1600k ns, 180°=2600k ns
  const int pan_min_duty  = std::stoi(get_param(p, "pwm_min_duty_ns_pan",     "600000"));
  const int pan_neutral   = std::stoi(get_param(p, "pwm_neutral_duty_ns_pan", "1600000"));
  const int pan_max_duty  = std::stoi(get_param(p, "pwm_max_duty_ns_pan",     "2600000"));

  // Tilt: 90°=1400k ns neutral; assuming ±1000k ns per 90°
  const int tilt_min_duty = std::stoi(get_param(p, "pwm_min_duty_ns_tilt",     "400000"));
  const int tilt_neutral  = std::stoi(get_param(p, "pwm_neutral_duty_ns_tilt", "1400000"));
  const int tilt_max_duty = std::stoi(get_param(p, "pwm_max_duty_ns_tilt",     "2400000"));

  // Angle limits (default ±2*pi/3 rad = ±120°, matching URDF joint limits)
  const double min_angle_rad = std::stod(get_param(p, "min_angle_rad", "-2.0943951"));
  const double max_angle_rad = std::stod(get_param(p, "max_angle_rad",  "2.0943951"));

  // Configure pan driver
  pan_driver_.set_period_ns(period_ns);
  pan_driver_.set_min_duty_ns(pan_min_duty);
  pan_driver_.set_neutral_duty_ns(pan_neutral);
  pan_driver_.set_max_duty_ns(pan_max_duty);
  pan_driver_.set_angle_range(min_angle_rad, max_angle_rad);

  // Configure tilt driver
  tilt_driver_.set_period_ns(period_ns);
  tilt_driver_.set_min_duty_ns(tilt_min_duty);
  tilt_driver_.set_neutral_duty_ns(tilt_neutral);
  tilt_driver_.set_max_duty_ns(tilt_max_duty);
  tilt_driver_.set_angle_range(min_angle_rad, max_angle_rad);

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured: mock=%s  pan=%s/%d [%d,%d,%d ns]  tilt=%s/%d [%d,%d,%d ns]",
    use_mock_ ? "true" : "false",
    chip_pan_.c_str(), channel_pan_, pan_min_duty, pan_neutral, pan_max_duty,
    chip_tilt_.c_str(), channel_tilt_, tilt_min_duty, tilt_neutral, tilt_max_duty);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// export_state_interfaces
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
PWMHeadSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joint 0 = pan, Joint 1 = tilt  (order matches URDF)
  state_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_state_pan_);
  state_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_POSITION, &hw_state_tilt_);

  return state_interfaces;
}

// ---------------------------------------------------------------------------
// export_command_interfaces
// ---------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
PWMHeadSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_cmd_pan_);
  command_interfaces.emplace_back(
    info_.joints[1].name, hardware_interface::HW_IF_POSITION, &hw_cmd_tilt_);

  return command_interfaces;
}

// ---------------------------------------------------------------------------
// on_activate
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn PWMHeadSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Activating head hardware...");

  if (!pan_driver_.init(chip_pan_, channel_pan_, use_mock_)) {
    RCLCPP_FATAL(rclcpp::get_logger(kLoggerName), "Failed to initialise pan PWM driver");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!tilt_driver_.init(chip_tilt_, channel_tilt_, use_mock_)) {
    RCLCPP_FATAL(rclcpp::get_logger(kLoggerName), "Failed to initialise tilt PWM driver");
    pan_driver_.shutdown();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read initial position from URDF state interface initial_value, default 0
  auto read_initial = [](const hardware_interface::ComponentInfo & joint) -> double {
      if (!joint.state_interfaces.empty() &&
        !joint.state_interfaces[0].initial_value.empty())
      {
        return std::stod(joint.state_interfaces[0].initial_value);
      }
      return 0.0;
    };

  const double init_pan  = read_initial(info_.joints[0]);
  const double init_tilt = read_initial(info_.joints[1]);

  hw_cmd_pan_   = init_pan;
  hw_cmd_tilt_  = init_tilt;
  hw_state_pan_ = init_pan;
  hw_state_tilt_ = init_tilt;

  pan_driver_.set_position_rad(init_pan);
  tilt_driver_.set_position_rad(init_tilt);

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Head hardware activated (pan=%.4f rad, tilt=%.4f rad)", init_pan, init_tilt);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn PWMHeadSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Deactivating head hardware...");

  pan_driver_.shutdown();
  tilt_driver_.shutdown();

  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Head hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// read  (open-loop: state = last command)
// ---------------------------------------------------------------------------
hardware_interface::return_type PWMHeadSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  hw_state_pan_  = pan_driver_.get_position_rad();
  hw_state_tilt_ = tilt_driver_.get_position_rad();
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write
// ---------------------------------------------------------------------------
hardware_interface::return_type PWMHeadSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  pan_driver_.set_position_rad(hw_cmd_pan_);
  tilt_driver_.set_position_rad(hw_cmd_tilt_);
  return hardware_interface::return_type::OK;
}

}  // namespace rhphumanoid_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rhphumanoid_hardware::PWMHeadSystemHardware,
  hardware_interface::SystemInterface)
