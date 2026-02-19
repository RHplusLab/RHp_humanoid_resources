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

#ifndef RHPHUMANOID_HARDWARE_INTERFACE__PWM_HEAD_SYSTEM_HPP_
#define RHPHUMANOID_HARDWARE_INTERFACE__PWM_HEAD_SYSTEM_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rhphumanoid_hardware_interface/pwm_driver.hpp"
#include "rhphumanoid_hardware_interface/visibility_control.hpp"

namespace rhphumanoid_hardware
{

/// ros2_control SystemInterface for driving two SG-90 servos (head pan + tilt)
/// via Linux sysfs PWM.  Includes a mock mode (default) that logs commands
/// instead of writing to sysfs, allowing development on hardware-less hosts.
///
/// URDF parameters (all optional, sensible defaults provided):
///   use_mock             : "true"/"false"     default true
///   pwm_chip_pan         : sysfs chip name    default "pwmchip3"
///   pwm_channel_pan      : channel number     default 0
///   pwm_chip_tilt        : sysfs chip name    default "pwmchip2"
///   pwm_channel_tilt     : channel number     default 0
///   pwm_period_ns        : period in ns       default 20000000  (50 Hz)
///   pwm_min_duty_ns_pan  : duty at -2π/3 rad  default 600000
///   pwm_neutral_duty_ns_pan : duty at 0 rad   default 1600000
///   pwm_max_duty_ns_pan  : duty at +2π/3 rad  default 2600000
///   pwm_min_duty_ns_tilt : duty at -2π/3 rad  default 400000
///   pwm_neutral_duty_ns_tilt: duty at 0 rad   default 1400000
///   pwm_max_duty_ns_tilt : duty at +2π/3 rad  default 2400000
class PWMHeadSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PWMHeadSystemHardware)

  RHPHUMANOID_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  RHPHUMANOID_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  RHPHUMANOID_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  RHPHUMANOID_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  RHPHUMANOID_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  RHPHUMANOID_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  RHPHUMANOID_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Per-joint PWM drivers
  PWMDriver pan_driver_;
  PWMDriver tilt_driver_;

  // Command / state storage (position only, open-loop)
  double hw_cmd_pan_{0.0};
  double hw_cmd_tilt_{0.0};
  double hw_state_pan_{0.0};
  double hw_state_tilt_{0.0};

  // Configuration
  bool use_mock_{true};
  std::string chip_pan_;
  int channel_pan_{0};
  std::string chip_tilt_;
  int channel_tilt_{0};
};

}  // namespace rhphumanoid_hardware

#endif  // RHPHUMANOID_HARDWARE_INTERFACE__PWM_HEAD_SYSTEM_HPP_
