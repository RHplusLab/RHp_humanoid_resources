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

#ifndef RHPHUMANOID_HARDWARE_INTERFACE__PWM_DRIVER_HPP_
#define RHPHUMANOID_HARDWARE_INTERFACE__PWM_DRIVER_HPP_

#include <string>

namespace rhphumanoid_hardware
{

/// Low-level driver for a single SG-90 servo via Linux sysfs PWM.
///
/// Supports a mock mode that logs all would-be sysfs writes via RCLCPP_INFO
/// instead of touching the filesystem.  Mock mode is the default.
class PWMDriver
{
public:
  PWMDriver() = default;
  ~PWMDriver() = default;

  /// Initialise the PWM channel.
  /// @param chip     sysfs chip name, e.g. "pwmchip3"
  /// @param channel  PWM channel number within the chip
  /// @param use_mock if true, log instead of writing sysfs
  /// @return true on success
  bool init(const std::string & chip, int channel, bool use_mock);

  /// Command the servo to a position in radians.
  /// The value is clamped to [min_angle_rad_, max_angle_rad_].
  void set_position_rad(double radians);

  /// Return the last commanded position (open-loop, no encoder feedback).
  double get_position_rad() const;

  /// Return the servo to neutral and disable the PWM output.
  void shutdown();

  // --------------- configuration setters (call before init) ---------------

  void set_period_ns(int period_ns) { period_ns_ = period_ns; }
  void set_min_duty_ns(int min_duty_ns) { min_duty_ns_ = min_duty_ns; }
  void set_max_duty_ns(int max_duty_ns) { max_duty_ns_ = max_duty_ns; }
  void set_neutral_duty_ns(int neutral_duty_ns) { neutral_duty_ns_ = neutral_duty_ns; }
  void set_angle_range(double min_rad, double max_rad)
  {
    min_angle_rad_ = min_rad;
    max_angle_rad_ = max_rad;
  }

private:
  /// Write a string value to a sysfs file.  Returns true on success.
  bool write_sysfs(const std::string & path, const std::string & value);

  /// Read a string value from a sysfs file.
  std::string read_sysfs(const std::string & path);

  /// Convert a radian angle to a duty-cycle value in nanoseconds.
  /// Linear mapping: min_angle_rad -> min_duty_ns, max_angle_rad -> max_duty_ns.
  int rad_to_duty_ns(double rad) const;

  // sysfs base path, e.g. "/sys/class/pwm/pwmchip3/pwm0/"
  std::string sysfs_base_path_;

  bool mock_mode_{true};
  bool initialized_{false};
  double last_position_rad_{0.0};

  // 50 Hz period
  int period_ns_{20000000};

  // Pan servo defaults: 0°=600k, 90°=1600k, 180°=2600k
  int min_duty_ns_{600000};
  int max_duty_ns_{2600000};
  int neutral_duty_ns_{1600000};

  // Joint angle limits (±2*pi/3 rad = ±120°)
  double min_angle_rad_{-2.0943951};
  double max_angle_rad_{ 2.0943951};
};

}  // namespace rhphumanoid_hardware

#endif  // RHPHUMANOID_HARDWARE_INTERFACE__PWM_DRIVER_HPP_
