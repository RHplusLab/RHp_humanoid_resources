#include "rhphumanoid_hardware_interface/rhphumanoid.hpp"
#include "rhphumanoid_hardware_interface/rhphumanoid_serial.hpp"

// C++ Standard Libraries
#include <algorithm>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <glob.h>
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
  constexpr auto UPDATE_PERIOD_IDLE = 100ms;

  constexpr int IDLE_ENTRY_CNT = 50;
  constexpr int FIRST_SET_MOVE_TIME = 2000;
  constexpr int READ_POS_SKIP_COUNT = 5;
  constexpr int ID_PROBE_TRIES = 3;
  constexpr auto STATS_PERIOD = 10s;

  // Manual mode configuration
  constexpr int UPDATE_CNT_CHK_FOR_MANUAL_MODE = (2000 / 100);
  const std::string MANUAL_MODE_ENABLE_FILE = "/tmp/rhphumanoid_enable_manual_mode";

  rclcpp::Logger logger() { return rclcpp::get_logger("RHPHumanoidSystemHardware"); }
}

namespace rhphumanoid
{

rhphumanoid::rhphumanoid()
: inited_(false),
  run_(false),
  gripper_pos_min_m_(0.0),
  gripper_pos_min_s_(0.0),
  gripper_pos_max_s_(0.0),
  gripper_pos_m_to_s_factor_(0.0)
{
}

rhphumanoid::~rhphumanoid()
{
  run_ = false;
  for (auto & bus : buses_) {
    if (bus->thread.joinable()) {
      bus->thread.join();
    }
  }
  for (auto & bus : buses_) {
    if (bus->drvr) {
      bus->drvr->close();
    }
  }
}

bool rhphumanoid::init()
{
  return init(BusConfig());
}

std::vector<std::string> rhphumanoid::resolvePorts(const std::vector<std::string> & ports)
{
  std::vector<std::string> out;
  for (const auto & p : ports) {
    if (p != "auto") {
      out.push_back(p);
      continue;
    }
    glob_t g;
    if (glob("/dev/ttyUSB*", 0, nullptr, &g) == 0) {
      for (size_t i = 0; i < g.gl_pathc; ++i) {
        out.emplace_back(g.gl_pathv[i]);
      }
    }
    globfree(&g);
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

bool rhphumanoid::init(const BusConfig & cfg)
{
  if (inited_) return false;
  cfg_ = cfg;

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

  RCLCPP_INFO(logger(), "Joint limits initialization:");

  for (const auto & [name, id] : joint_defs) {
    joint_name_map_[name] = id;

		// [수정된 부분] -----------------------------------------------------
    // 1. 기본 Limit 설정을 복사합니다.
    JointLimit current_limit = DEFAULT_LIMIT;

    // 2. r_wst 관절인 경우에만 방향(invert_factor)을 -1로 변경합니다.
    if (name == "r_wst") {
        current_limit.invert_factor = -1;
    }

    // 3. 수정된 설정을 맵에 저장합니다.
    joint_range_limits_[name] = current_limit;
    // ------------------------------------------------------------------

    last_pos_set_map_[name] = {INVALID_POS, false};
    last_pos_get_map_[name] = {INVALID_POS, false};

    RCLCPP_INFO(logger(),
      "Joint: %s, ID: %d, min/max(rad): %.4f, %.4f",
      name.c_str(), id,
      jointValueToPosition(name, DEFAULT_LIMIT.min),
      jointValueToPosition(name, DEFAULT_LIMIT.max));
  }

  // ===== Open every serial bus =====
  const auto ports = resolvePorts(cfg_.ports);
  if (ports.empty()) {
    RCLCPP_ERROR(logger(), "No serial port to open (serial_ports parameter / no /dev/ttyUSB*)");
    return false;
  }
  for (const auto & port : ports) {
    auto bus = std::make_unique<Bus>();
    bus->port = port;
    bus->drvr = std::make_unique<rhphumanoid_serial>();
    if (!bus->drvr->open(port)) {
      RCLCPP_ERROR(logger(), "Failed to open serial port %s", port.c_str());
      continue;
    }
    buses_.push_back(std::move(bus));
  }

  // ===== Initial position read: also finds which bus each servo is on =====
  RCLCPP_INFO(logger(), "Reading ALL joint positions on %zu bus(es)...", buses_.size());
  std::vector<std::string> failed_joints;
  std::vector<std::string> duplicate_joints;
  for (const auto & [name, id] : joint_name_map_) {
    std::vector<Bus *> hits;
    int pos = INVALID_POS;
    for (auto & bus : buses_) {
      for (int t = 0; t < ID_PROBE_TRIES; ++t) {
        int p = 0;
        if (bus->drvr->readPosition(id, p, cfg_.read_timeout_ms)) {
          hits.push_back(bus.get());
          pos = p;
          break;
        }
      }
    }
    if (hits.empty()) {
      failed_joints.push_back("ID: " + std::to_string(id) + " (" + name + ")");
    } else if (hits.size() > 1) {
      duplicate_joints.push_back("ID: " + std::to_string(id) + " (" + name + ") on " + hits[0]->port + " and " + hits[1]->port);
    } else {
      joint_bus_[name] = hits[0];
      hits[0]->joints.push_back(name);
      hits[0]->ids.push_back(id);
      last_pos_get_map_[name] = {pos, true};
    }
  }

  // =========================================================================
  // [ADDED FEATURE] Check for disconnected motors and abort if any failed
  // =========================================================================
  if (!failed_joints.empty() || !duplicate_joints.empty()) {
    RCLCPP_FATAL(logger(), "============================================");
    if (!failed_joints.empty()) {
      RCLCPP_FATAL(logger(), "!!! MOTOR COMMUNICATION FAILURE DETECTED !!!");
      RCLCPP_FATAL(logger(), "The following motors are NOT responding:");
      for (const auto & error_msg : failed_joints) {
        RCLCPP_FATAL(logger(), "  -> %s", error_msg.c_str());
      }
      RCLCPP_FATAL(logger(), "Please check the wiring for the motors listed above.");
    }
    if (!duplicate_joints.empty()) {
      RCLCPP_FATAL(logger(), "!!! THE SAME SERVO ID ANSWERS ON TWO BUSES !!!");
      for (const auto & error_msg : duplicate_joints) {
        RCLCPP_FATAL(logger(), "  -> %s", error_msg.c_str());
      }
    }
    RCLCPP_FATAL(logger(), "System initialization ABORTED to prevent damage.");
    RCLCPP_FATAL(logger(), "============================================");

    // Stop initialization here
    return false;
  }
  // =========================================================================

  // drop ports where no servo answered (another USB serial device)
  buses_.erase(std::remove_if(buses_.begin(), buses_.end(), [](const std::unique_ptr<Bus> & b) {
    if (b->joints.empty()) {
      RCLCPP_WARN(logger(), "No servo answered on %s: port not used", b->port.c_str());
      b->drvr->close();
      return true;
    }
    return false;
  }), buses_.end());

  for (const auto & bus : buses_) {
    std::string ids;
    for (int id : bus->ids) ids += std::to_string(id) + " ";
    RCLCPP_INFO(logger(), "Bus %s: %zu servos (IDs %s)", bus->port.c_str(), bus->ids.size(), ids.c_str());
  }
  RCLCPP_INFO(logger(), "Position read mode: %s",
    cfg_.continuous_read ? "CONTINUOUS (state = measured position)" : "INITIAL 2 s, then write-only (state = last command)");

  run_ = true;
  for (auto & bus : buses_) {
    Bus * b = bus.get();
    if (cfg_.continuous_read) {
      bus->thread = std::thread([this, b]() { ProcessContinuous(*b); });
    } else {
      bus->thread = std::thread([this, b]() { Process(*b); });
    }
  }

  inited_ = true;
  return true;
}

void rhphumanoid::setAllJointPositions(const std::vector<double> & commands, const std::vector<std::string> & joints)
{
  if (commands.empty() || commands.size() != joints.size()) return;

  for (double cmd : commands) {
    if (!std::isfinite(cmd)) {
      // Use standard error logging since get_clock() might not be available or reliable in this context
      RCLCPP_WARN(logger(), "NaN or Infinite command detected. Ignoring all.");
      return;
    }
  }

  std::lock_guard<std::mutex> guard(mutex_);

  for (size_t i = 0; i < commands.size(); ++i) {
    const std::string & name = joints[i];

    auto bit = joint_bus_.find(name);
    if (bit == joint_bus_.end()) continue;

    int joint_pos = positionToJointValue(name, commands[i]);

    if (joint_pos != last_pos_set_map_[name].pos) {
      last_pos_set_map_[name] = {joint_pos, true};
      if (!cfg_.continuous_read) {
        // write-only mode: the reported state is the last command
        last_pos_get_map_[name] = {joint_pos, false};
      }
      bit->second->new_cmd = true;
    }
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
  auto it = joint_range_limits_.find(joint_name);
  if (it == joint_range_limits_.end()) {
    return 0;
  }
  const auto & limit = it->second;
  double range = static_cast<double>(limit.max - limit.min);
  return static_cast<int>((range * rad / limit.range_rad * limit.invert_factor) + limit.mid);
}

double rhphumanoid::convertUnitToRad(const std::string & joint_name, int unit)
{
  auto it = joint_range_limits_.find(joint_name);
  if (it == joint_range_limits_.end()) {
    return 0.0;
  }
  const auto & limit = it->second;
  double range = static_cast<double>(limit.max - limit.min);
  return (static_cast<double>(unit - limit.mid) * limit.range_rad * limit.invert_factor) / range;
}

double rhphumanoid::jointValueToPosition(const std::string & joint_name, int jointValue)
{
  return convertUnitToRad(joint_name, jointValue);
}

int rhphumanoid::positionToJointValue(const std::string & joint_name, double position)
{
  return convertRadToUnit(joint_name, position);
}

void rhphumanoid::readJointPositions(Bus & bus, std::map<std::string, JointState> & pos_map)
{
  for (size_t i = 0; i < bus.joints.size(); ++i) {
    int p = 0;
    if (!bus.drvr->readPosition(bus.ids[i], p, cfg_.read_timeout_ms)) {
      bus.read_failures++;
      RCLCPP_WARN(logger(), "Failed to read joint: %d (%s)", bus.ids[i], bus.joints[i].c_str());
      continue;
    }
    pos_map[bus.joints[i]] = {p, true};
  }
}

void rhphumanoid::writeChanged(Bus & bus, const std::map<std::string, JointState> & cmd_snapshot, int move_time)
{
  std::vector<std::pair<int, int>> items;
  for (size_t i = 0; i < bus.joints.size(); ++i) {
    auto it = cmd_snapshot.find(bus.joints[i]);
    if (it != cmd_snapshot.end() && it->second.changed) {
      items.emplace_back(bus.ids[i], it->second.pos);
    }
  }
  if (!items.empty()) {
    bus.drvr->movePositions(items, static_cast<uint16_t>(move_time));
  }
}

bool rhphumanoid::manual_mode_enabled()
{
  return std::filesystem::exists(MANUAL_MODE_ENABLE_FILE);
}

void rhphumanoid::set_manual_mode(Bus & bus, bool enable)
{
  RCLCPP_INFO(logger(), "Setting Manual Mode on %s: %s", bus.port.c_str(), enable ? "ON" : "OFF");
  if (!bus.drvr->setManualMode(bus.ids, enable)) {
    RCLCPP_ERROR(logger(), "Failed to set manual mode");
  }
}

// Previous behaviour, now per bus: read for the first 2 s, then write-only; idle period 100 ms.
void rhphumanoid::Process(Bus & bus)
{
  const auto update_period_moving = std::chrono::milliseconds(cfg_.update_period_ms);

  int ck_for_manual_mode_cnt = 0;
  bool manual_mode = false;

  bool idle = false;
  int ck_for_idle_cnt = 0;

  bool first_set = true;
  int initial_read_counter = 20; // 수정됨: 20회 * 5루프 * 20ms = 정확히 2000ms

  int read_pos_delay_cnt = 0;
  std::map<std::string, JointState> local_pos_map;

  // Initial time setup before loop
  auto next_update_time = std::chrono::steady_clock::now();

  while (run_) {
    // Determine the period based on idle state
    auto period = idle ? std::chrono::milliseconds(UPDATE_PERIOD_IDLE) : update_period_moving;
    next_update_time += period;

    // 1. Check Manual Mode Logic
    if (idle && --ck_for_manual_mode_cnt <= 0) {
      ck_for_manual_mode_cnt = UPDATE_CNT_CHK_FOR_MANUAL_MODE;
      bool enabled_file = manual_mode_enabled();

      if (manual_mode != enabled_file) {
        set_manual_mode(bus, enabled_file);
        manual_mode = enabled_file;
      }
    }

    // 2. Handle Commands
    bool has_new_cmd = false;
    std::map<std::string, JointState> cmd_snapshot;

    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (bus.new_cmd) {
        for (const auto & name : bus.joints) {
          cmd_snapshot[name] = last_pos_set_map_[name];
          last_pos_set_map_[name].changed = false;
        }
        has_new_cmd = true;
        bus.new_cmd = false;
      }
    }

    if (has_new_cmd) {
      read_pos_delay_cnt = 1;

      int move_time = first_set ? FIRST_SET_MOVE_TIME : cfg_.update_period_ms;
      writeChanged(bus, cmd_snapshot, move_time);

      first_set = false;

      if (idle) {
        RCLCPP_INFO(logger(), "Exiting Idle Mode -> Active (%s)", bus.port.c_str());
        idle = false;
      }

      ck_for_idle_cnt = 0;
      ck_for_manual_mode_cnt = 0;

    } else {
      // 수정됨: initial_read_counter가 0 이하일 때만 Idle 모드로 진입 허용
      if (!idle && initial_read_counter <= 0 && ++ck_for_idle_cnt > IDLE_ENTRY_CNT) {
        idle = true;
        RCLCPP_INFO(logger(), "Entering Idle Mode (%s)", bus.port.c_str());
      }
    }

    // 3. Hardware Initial Read Check & Bringup Tracking
    if (!has_new_cmd && --read_pos_delay_cnt <= 0) {
      read_pos_delay_cnt = READ_POS_SKIP_COUNT;

      if (initial_read_counter > 0) {
        if (initial_read_counter == 20) { // 수정됨
          RCLCPP_INFO(logger(), "Performing INITIAL hardware check & tracking movement for 2s (%s)...", bus.port.c_str());
        }

        {
          std::lock_guard<std::mutex> guard(mutex_);
          local_pos_map.clear();
          for (const auto & name : bus.joints) local_pos_map[name] = last_pos_get_map_[name];
        }

        readJointPositions(bus, local_pos_map);

        {
          std::lock_guard<std::mutex> guard(mutex_);

          if (!bus.new_cmd)
          {
            for (const auto & kv : local_pos_map) last_pos_get_map_[kv.first] = kv.second;
          }
          else
          {
            RCLCPP_WARN(logger(),
                        "Race condition detected! New command received during read. Discarding read result.");
          }
        }

        initial_read_counter--;

        if (initial_read_counter == 0) {
          RCLCPP_INFO(logger(), "Initial 2s tracking DONE. Switching to WRITE-ONLY mode (%s).", bus.port.c_str());
        }
      }
    }

    std::this_thread::sleep_until(next_update_time);
  }
}

// Continuous mode (e.g. RL policies that need measured joint positions): every cycle write the changed targets in one
// write, then read reads_per_cycle servos of this bus (round robin, 0 = all). No idle mode.
void rhphumanoid::ProcessContinuous(Bus & bus)
{
  const auto period = std::chrono::milliseconds(cfg_.update_period_ms);
  const size_t n_read = (cfg_.reads_per_cycle <= 0) ? bus.joints.size()
                        : std::min(bus.joints.size(), static_cast<size_t>(cfg_.reads_per_cycle));
  size_t rr = 0;
  bool first_set = true;
  bool manual_mode = false;
  auto next_manual_check = std::chrono::steady_clock::now();
  auto next_stats = std::chrono::steady_clock::now() + STATS_PERIOD;
  long cycles = 0;
  double cycle_sum_ms = 0.0;

  auto next_update_time = std::chrono::steady_clock::now();

  while (run_) {
    const auto t0 = std::chrono::steady_clock::now();
    next_update_time += period;

    // 1. Manual mode (file check every 2 s)
    if (t0 >= next_manual_check) {
      next_manual_check = t0 + 2s;
      bool enabled_file = manual_mode_enabled();
      if (manual_mode != enabled_file) {
        set_manual_mode(bus, enabled_file);
        manual_mode = enabled_file;
      }
    }

    // 2. Write changed targets
    std::map<std::string, JointState> cmd_snapshot;
    bool has_new_cmd = false;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (bus.new_cmd) {
        for (const auto & name : bus.joints) {
          cmd_snapshot[name] = last_pos_set_map_[name];
          last_pos_set_map_[name].changed = false;
        }
        has_new_cmd = true;
        bus.new_cmd = false;
      }
    }
    if (has_new_cmd && !manual_mode) {
      writeChanged(bus, cmd_snapshot, first_set ? FIRST_SET_MOVE_TIME : cfg_.update_period_ms);
      first_set = false;
    }

    // 3. Read measured positions
    for (size_t k = 0; k < n_read && run_; ++k) {
      const size_t i = rr++ % bus.joints.size();
      int p = 0;
      if (bus.drvr->readPosition(bus.ids[i], p, cfg_.read_timeout_ms)) {
        std::lock_guard<std::mutex> guard(mutex_);
        last_pos_get_map_[bus.joints[i]] = {p, true};
      } else {
        bus.read_failures++;
      }
    }

    const double dt_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    bus.cycle_ms = dt_ms;
    cycles++;
    cycle_sum_ms += dt_ms;
    if (std::chrono::steady_clock::now() >= next_stats) {
      next_stats += STATS_PERIOD;
      RCLCPP_INFO(logger(), "Bus %s: %.1f cycles/s, busy %.1f ms per cycle, %zu/%zu servos read per cycle, read failures %ld",
        bus.port.c_str(), cycles / std::chrono::duration<double>(STATS_PERIOD).count(), cycle_sum_ms / cycles, n_read,
        bus.joints.size(), bus.read_failures.load());
      cycles = 0;
      cycle_sum_ms = 0.0;
    }

    // a cycle longer than the period starts the next one immediately (no catch-up burst)
    const auto now = std::chrono::steady_clock::now();
    if (next_update_time < now) {
      next_update_time = now;
    } else {
      std::this_thread::sleep_until(next_update_time);
    }
  }
}

} // namespace rhphumanoid
