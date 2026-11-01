#ifndef RHPHUMANOID_HARDWARE_INTERFACE__RHPHUMANOID_HPP_
#define RHPHUMANOID_HARDWARE_INTERFACE__RHPHUMANOID_HPP_

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace rhphumanoid
{

// Forward declaration MUST be inside the namespace
class rhphumanoid_serial;
// class rhphumanoid_usb; // If needed later

struct JointLimit
{
  double range_rad;
  int min;
  int max;
  int mid;
  int invert_factor; // 1 or -1
};

struct JointState
{
  int pos;
  bool changed;
};

// Serial bus settings (ros2_control hardware parameters, see rhphumanoid_system.cpp)
struct BusConfig
{
  // serial devices; "auto" = every /dev/ttyUSB*. Which servo is on which bus is found at init by asking every ID.
  std::vector<std::string> ports{"/dev/ttyRHP"};
  // false: read positions only for the first 2 s, then write-only (state = last command; previous behaviour)
  // true:  keep reading measured positions every cycle (state = measured position)
  bool continuous_read = false;
  int reads_per_cycle = 0;    // continuous_read: servos read per bus per cycle, round robin (0 = all of the bus)
  int update_period_ms = 20;  // bus loop period while moving
  // one position read. Measured 2026-10-29 (1 bus): WCH ch34x driver ~23.5 ms, Linux ch341 ~3.6 ms; a read right
  // after a batch write also waits for the queued write bytes (11 servos: ~7 ms)
  int read_timeout_ms = 50;
};

class rhphumanoid
{
public:
  rhphumanoid();
  virtual ~rhphumanoid();

  bool init();
  bool init(const BusConfig & cfg);

  // Joint Command Interface
  void setAllJointPositions(const std::vector<double> & commands, const std::vector<std::string> & joints);
  void getAllJointPositions(std::vector<double> & positions, const std::vector<std::string> & joints);

private:
  struct Bus
  {
    std::string port;
    std::unique_ptr<rhphumanoid_serial> drvr;
    std::vector<std::string> joints;  // joints whose servo answered on this bus
    std::vector<int> ids;
    std::thread thread;
    bool new_cmd = false;             // guarded by mutex_
    std::atomic<double> cycle_ms{0.0};
    std::atomic<long> read_failures{0};
  };

  // Thread functions (one thread per bus)
  void Process(Bus & bus);            // write-only after the initial 2 s (continuous_read = false)
  void ProcessContinuous(Bus & bus);  // write + read every cycle (continuous_read = true)

  // Internal Helpers
  double jointValueToPosition(const std::string & joint_name, int jointValue);
  int positionToJointValue(const std::string & joint_name, double position);
  int convertRadToUnit(const std::string & joint_name, double rad);
  double convertUnitToRad(const std::string & joint_name, int unit);

  std::vector<std::string> resolvePorts(const std::vector<std::string> & ports);
  void readJointPositions(Bus & bus, std::map<std::string, JointState> & pos_map);
  void writeChanged(Bus & bus, const std::map<std::string, JointState> & cmd_snapshot, int move_time);

  bool manual_mode_enabled();
  void set_manual_mode(Bus & bus, bool enable);

  // Members (Declaration order matches initialization order to prevent -Wreorder)
  bool inited_;
  std::atomic<bool> run_;

  BusConfig cfg_;
  std::vector<std::unique_ptr<Bus>> buses_;
  std::map<std::string, Bus *> joint_bus_;

  std::mutex mutex_;

  // Mappings
  std::map<std::string, int> joint_name_map_;
  std::map<std::string, JointLimit> joint_range_limits_;

  // State Storage
  std::map<std::string, JointState> last_pos_set_map_;
  std::map<std::string, JointState> last_pos_get_map_;

  // Gripper params
  double gripper_pos_min_m_;
  double gripper_pos_min_s_;
  double gripper_pos_max_s_;
  double gripper_pos_m_to_s_factor_;
};

} // namespace rhphumanoid

#endif // RHPHUMANOID_HARDWARE_INTERFACE__RHPHUMANOID_HPP_
