#include "rhphumanoid_hardware_interface/rhphumanoid.hpp"
#include "rhphumanoid_hardware_interface/rhphumanoid_serial.hpp"

// C++ Standard Libraries
#include <cmath>
#include <chrono>
#include <filesystem>
#include <vector>
#include <string>
#include <utility>

// ROS2
#include "rclcpp/rclcpp.hpp"

// Constants definition
namespace {
  constexpr double RAD_RANGE = (240.0 / 180.0) * M_PI;
  constexpr int INVALID_POS = 99999;

  // Timing configuration
  using namespace std::chrono_literals;
  constexpr auto UPDATE_PERIOD_MOVING = 10ms;
  constexpr auto UPDATE_PERIOD_IDLE = 100ms;

  constexpr int IDLE_ENTRY_CNT = 50;
  constexpr int FIRST_SET_MOVE_TIME = 1500;

  // Manual mode configuration
  constexpr int UPDATE_CNT_CHK_FOR_MANUAL_MODE = (2000 / 100);
  const std::string MANUAL_MODE_ENABLE_FILE = "/tmp/rhphumanoid_enable_manual_mode";

  constexpr int NUM_JOINTS = 22;
  const std::string SERIAL_DEV = "/dev/ttyRHP";
}

namespace rhphumanoid
{

rhphumanoid::rhphumanoid()
: inited_(false),
  run_(false),
  gripper_pos_min_m_(0.0),
  gripper_pos_min_s_(0.0),
  gripper_pos_max_s_(0.0),
  gripper_pos_m_to_s_factor_(0.0),
  new_cmd_(false)
{
}

rhphumanoid::~rhphumanoid()
{
  if (inited_) {
    run_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  if (drvr_) {
    drvr_->close();
  }
}

bool rhphumanoid::init()
{
  if (inited_) return false;

  std::string dev = SERIAL_DEV;

#if defined(RHPHUMANOID_USB)
  drvr_ = std::make_unique<rhphumanoid_usb>();
#else
  drvr_ = std::make_unique<rhphumanoid_serial>();
#endif

  if (!drvr_) return false;

  if (!drvr_->open(dev)) {
    RCLCPP_ERROR(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Failed to open driver");
    return false;
  }

  // ===== Optimization: Data Driven Initialization =====
  const JointLimit DEFAULT_LIMIT = {RAD_RANGE, 0, 1000, 500, 1};

  const std::vector<std::pair<std::string, int>> joint_defs = {
    {"l_hip_yaw", 1},   {"l_hip_roll", 2},   {"l_hip_pitch", 3},
    {"l_knee", 4},      {"l_ank_pitch", 5},  {"l_ank_roll", 6},
    {"r_hip_yaw", 7},   {"r_hip_roll", 8},   {"r_hip_pitch", 9},
    {"r_knee", 10},     {"r_ank_pitch", 11}, {"r_ank_roll", 12},

    {"l_sho_pitch", 13},{"l_sho_roll", 14},  {"l_el", 15},
    {"l_wst", 16},      {"l_grp", 17},
    {"r_sho_pitch", 18},{"r_sho_roll", 19},  {"r_el", 20},
    {"r_wst", 21},      {"r_grp", 22}
  };

  RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Joint limits initialization:");

  for (const auto & [name, id] : joint_defs) {
    joint_name_map_[name] = id;
    joint_range_limits_[name] = DEFAULT_LIMIT;

    last_pos_set_map_[name] = {INVALID_POS, false};
    last_pos_get_map_[name] = {INVALID_POS, false};

    RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"),
      "Joint: %s, ID: %d, min/max(rad): %.4f, %.4f",
      name.c_str(), id,
      jointValueToPosition(name, DEFAULT_LIMIT.min),
      jointValueToPosition(name, DEFAULT_LIMIT.max));
  }

  // Initial position read
  readJointPositions(last_pos_get_map_);

  run_ = true;
  thread_ = std::thread(&rhphumanoid::Process, this);

  inited_ = true;
  return true;
}

void rhphumanoid::setAllJointPositions(const std::vector<double> & commands, const std::vector<std::string> & joints)
{
  if (commands.empty() || commands.size() != joints.size()) return;
  if (!std::isfinite(commands[0])) {
    std::lock_guard<std::mutex> guard(mutex_);
    new_cmd_ = false;
    return;
  }

  std::lock_guard<std::mutex> guard(mutex_);

  bool any_change = false;
  for (size_t i = 0; i < commands.size(); ++i) {
    const std::string & name = joints[i];

    if (joint_name_map_.find(name) == joint_name_map_.end()) continue;

    int joint_pos = positionToJointValue(name, commands[i]);

    if (joint_pos != last_pos_set_map_[name].pos) {
      last_pos_set_map_[name] = {joint_pos, true};
      last_pos_get_map_[name] = {joint_pos, false};
      any_change = true;
    }
  }

  if (any_change) {
    new_cmd_ = true;
  }
}

void rhphumanoid::getAllJointPositions(std::vector<double> & positions, const std::vector<std::string> & joints)
{
  std::lock_guard<std::mutex> guard(mutex_);
  positions.reserve(joints.size());

  for (const auto & name : joints) {
    if (last_pos_get_map_.find(name) == last_pos_get_map_.end()) {
        positions.push_back(0.0);
        continue;
    }
    double pos_rad = jointValueToPosition(name, last_pos_get_map_[name].pos);
    positions.push_back(pos_rad);
  }
}

int rhphumanoid::convertRadToUnit(const std::string & joint_name, double rad)
{
  try {
    const auto & limit = joint_range_limits_.at(joint_name);
    double range = static_cast<double>(limit.max - limit.min);
    return static_cast<int>((range * rad / limit.range_rad * limit.invert_factor) + limit.mid);
  } catch (...) {
    return 0;
  }
}

double rhphumanoid::convertUnitToRad(const std::string & joint_name, int unit)
{
  try {
    const auto & limit = joint_range_limits_.at(joint_name);
    double range = static_cast<double>(limit.max - limit.min);
    return (static_cast<double>(unit - limit.mid) * limit.range_rad * limit.invert_factor) / range;
  } catch (...) {
    return 0.0;
  }
}

double rhphumanoid::jointValueToPosition(const std::string & joint_name, int jointValue)
{
  return convertUnitToRad(joint_name, jointValue);
}

int rhphumanoid::positionToJointValue(const std::string & joint_name, double position)
{
  return convertRadToUnit(joint_name, position);
}

void rhphumanoid::readJointPositions(std::map<std::string, JointState> & pos_map)
{
  RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Reading ALL joint positions...");

  for (const auto & [name, id] : joint_name_map_) {
    uint16_t p = 0;
    if (!drvr_->getJointPosition(id, p)) {
      RCLCPP_WARN(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Failed to read joint: %d (%s)", id, name.c_str());
      continue;
    }

    pos_map[name] = {static_cast<int>(p), true};
    RCLCPP_DEBUG(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Read %s: %d", name.c_str(), p);
  }
}

void rhphumanoid::setJointPosition(const std::string & joint_name, int position, int time)
{
  auto it = joint_name_map_.find(joint_name);
  if (it == joint_name_map_.end()) return;

  if (!drvr_->setJointPosition(it->second, position, time)) {
    // Reverted from THROTTLE to standard ERROR to avoid get_clock() issues in non-node class
    RCLCPP_ERROR(rclcpp::get_logger("RHPHumanoidSystemHardware"),
      "Failed to set joint position for %s", joint_name.c_str());
  }
}

bool rhphumanoid::manual_mode_enabled()
{
  return std::filesystem::exists(MANUAL_MODE_ENABLE_FILE);
}

void rhphumanoid::set_manual_mode(bool enable)
{
  RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Setting Manual Mode: %s", enable ? "ON" : "OFF");
  if (!drvr_->setManualModeAll(enable, NUM_JOINTS)) {
    RCLCPP_ERROR(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Failed to set manual mode");
  }
}

void rhphumanoid::Process()
{
  int ck_for_manual_mode_cnt = 0;
  bool manual_mode = false;

  bool idle = false;
  int ck_for_idle_cnt = 0;

  bool first_set = true;
  bool initial_read_done = false;

  int read_pos_delay_cnt = 0;
  std::map<std::string, JointState> local_pos_map;

  while (run_) {
    auto next_update_time = std::chrono::steady_clock::now();

    // 1. Check Manual Mode Logic
    if (idle && --ck_for_manual_mode_cnt <= 0) {
      ck_for_manual_mode_cnt = UPDATE_CNT_CHK_FOR_MANUAL_MODE;
      bool enabled_file = manual_mode_enabled();

      if (manual_mode != enabled_file) {
        set_manual_mode(enabled_file);
        manual_mode = enabled_file;
      }
    }

    // 2. Handle Commands
    bool has_new_cmd = false;
    std::map<std::string, JointState> cmd_snapshot;

    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (new_cmd_) {
        cmd_snapshot = last_pos_set_map_;
        has_new_cmd = true;
        new_cmd_ = false;

        for (auto & kv : last_pos_set_map_) kv.second.changed = false;
      }
    }

    if (has_new_cmd) {
      read_pos_delay_cnt = 1;

      int move_time = first_set ? FIRST_SET_MOVE_TIME : static_cast<int>(UPDATE_PERIOD_MOVING.count());

      for (const auto & [name, state] : cmd_snapshot) {
        if (state.changed) {
          setJointPosition(name, state.pos, move_time);
        }
      }

      first_set = false;

      if (idle) {
        RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Exiting Idle Mode -> Active");
        idle = false;
      }

      ck_for_idle_cnt = 0;
      ck_for_manual_mode_cnt = 0;

    } else {
      if (!idle && ++ck_for_idle_cnt > IDLE_ENTRY_CNT) {
        idle = true;
        RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Entering Idle Mode");
      }
    }

    // 3. Hardware Initial Read Check
    if (!has_new_cmd && --read_pos_delay_cnt <= 0) {
      read_pos_delay_cnt = 5;

      if (!initial_read_done) {
        RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Performing INITIAL hardware check...");

        {
          std::lock_guard<std::mutex> guard(mutex_);
          local_pos_map = last_pos_get_map_;
        }

        readJointPositions(local_pos_map);

        {
          std::lock_guard<std::mutex> guard(mutex_);
          last_pos_get_map_ = local_pos_map;
        }

        initial_read_done = true;
        RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Initial check DONE. Switching to WRITE-ONLY mode.");
      }
    }

    auto period = idle ? UPDATE_PERIOD_IDLE : UPDATE_PERIOD_MOVING;
    next_update_time += period;
    std::this_thread::sleep_until(next_update_time);
  }
}

} // namespace rhphumanoid
