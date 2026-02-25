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

#include "rhphumanoid_hardware_interface/pwm_driver.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace rhphumanoid_hardware
{

static const char * kLoggerName = "PWMDriver";

bool PWMDriver::init(const std::string & chip, int channel, bool use_mock)
{
  mock_mode_ = use_mock;

  // Build the sysfs base path: /sys/class/pwm/<chip>/pwm<channel>/
  sysfs_base_path_ =
    "/sys/class/pwm/" + chip + "/pwm" + std::to_string(channel) + "/";

  const std::string export_path = "/sys/class/pwm/" + chip + "/export";

  if (mock_mode_) {
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "[MOCK] init: chip=%s  channel=%d  base_path=%s",
      chip.c_str(), channel, sysfs_base_path_.c_str());
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "[MOCK] Would export channel %d via %s", channel, export_path.c_str());
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "[MOCK] Would set period=%d ns, duty=%d ns (neutral), enable=1",
      period_ns_, neutral_duty_ns_);
  } else {
    std::ifstream test(sysfs_base_path_ + "period");
    if (test.good()) {
      // 이미 디렉토리(채널)가 존재함
      RCLCPP_INFO(
        rclcpp::get_logger(kLoggerName),
        "PWM channel %d on %s already exported, continuing", channel, chip.c_str());
    } else {
      // 존재하지 않을 때만 export 시도
      if (!write_sysfs(export_path, std::to_string(channel))) {
        RCLCPP_ERROR(
          rclcpp::get_logger(kLoggerName),
          "Failed to export PWM channel %d on %s and sysfs path does not exist",
          channel, chip.c_str());
        return false;
      }
    }

    // Set period
    if (!write_sysfs(sysfs_base_path_ + "period", std::to_string(period_ns_))) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Failed to set period");
      return false;
    }

    // Set duty cycle to neutral
    if (!write_sysfs(sysfs_base_path_ + "duty_cycle", std::to_string(neutral_duty_ns_))) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Failed to set initial duty_cycle");
      return false;
    }

    // Enable
    if (!write_sysfs(sysfs_base_path_ + "enable", "1")) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Failed to enable PWM");
      return false;
    }
  }

  last_position_rad_ = 0.0;  // neutral
  initialized_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "PWM driver initialised [%s]: angle range [%.4f, %.4f] rad, "
    "duty range [%d, %d] ns, period %d ns",
    mock_mode_ ? "MOCK" : "LIVE",
    min_angle_rad_, max_angle_rad_,
    min_duty_ns_, max_duty_ns_, period_ns_);

  return true;
}

void PWMDriver::set_position_rad(double radians)
{
  if (!initialized_) {
    RCLCPP_WARN(rclcpp::get_logger(kLoggerName), "set_position_rad called before init");
    return;
  }

  // Clamp to configured joint limits
  const double clamped = std::clamp(radians, min_angle_rad_, max_angle_rad_);
  if (clamped != radians) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName),
      "Commanded position %.4f rad clamped to %.4f rad", radians, clamped);
  }

  const int duty_ns = rad_to_duty_ns(clamped);

  if (mock_mode_) {
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "[MOCK] set_position_rad: %.4f rad -> duty_cycle=%d ns  (path: %sduty_cycle)",
      clamped, duty_ns, sysfs_base_path_.c_str());
  } else {
    if (!write_sysfs(sysfs_base_path_ + "duty_cycle", std::to_string(duty_ns))) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Failed to write duty_cycle");
    }
  }

  last_position_rad_ = clamped;
}

double PWMDriver::get_position_rad() const
{
  return last_position_rad_;
}

void PWMDriver::shutdown()
{
  if (!initialized_) {
    return;
  }

  if (mock_mode_) {
    RCLCPP_INFO(
      rclcpp::get_logger(kLoggerName),
      "[MOCK] shutdown: would set duty=%d ns (neutral), disable, unexport",
      neutral_duty_ns_);
  } else {
    // Return to neutral
    write_sysfs(sysfs_base_path_ + "duty_cycle", std::to_string(neutral_duty_ns_));
    // Disable
    write_sysfs(sysfs_base_path_ + "enable", "0");

    RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "PWM channel disabled");
  }

  initialized_ = false;
  last_position_rad_ = 0.0;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool PWMDriver::write_sysfs(const std::string & path, const std::string & value)
{
  std::ofstream file(path);
  if (!file.is_open()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Cannot open sysfs file for writing: %s", path.c_str());
    return false;
  }
  file << value;
  file.close();
  if (file.fail()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Write failed on sysfs file: %s (value: %s)", path.c_str(), value.c_str());
    return false;
  }
  return true;
}

std::string PWMDriver::read_sysfs(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Cannot open sysfs file for reading: %s", path.c_str());
    return {};
  }
  std::string value;
  std::getline(file, value);
  return value;
}

int PWMDriver::rad_to_duty_ns(double rad) const
{
  // Linear interpolation:
  //   min_angle_rad_ -> min_duty_ns_
  //   max_angle_rad_ -> max_duty_ns_
  //
  // duty = min_duty + (rad - min_angle) * (max_duty - min_duty) / (max_angle - min_angle)

  const double range_rad = max_angle_rad_ - min_angle_rad_;
  if (std::abs(range_rad) < 1e-9) {
    return neutral_duty_ns_;
  }

  const double fraction = (rad - min_angle_rad_) / range_rad;
  const double duty = min_duty_ns_ + fraction * (max_duty_ns_ - min_duty_ns_);

  // Clamp to physical limits
  return std::clamp(static_cast<int>(std::round(duty)), min_duty_ns_, max_duty_ns_);
}

}  // namespace rhphumanoid_hardware
