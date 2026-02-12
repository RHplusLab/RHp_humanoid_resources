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

class rhphumanoid
{
public:
  rhphumanoid();
  virtual ~rhphumanoid();

  bool init();

  // Thread function
  void Process();

  // Joint Command Interface
  void setAllJointPositions(const std::vector<double> & commands, const std::vector<std::string> & joints);
  void getAllJointPositions(std::vector<double> & positions, const std::vector<std::string> & joints);

private:
  // Internal Helpers
  double jointValueToPosition(const std::string & joint_name, int jointValue);
  int positionToJointValue(const std::string & joint_name, double position);
  int convertRadToUnit(const std::string & joint_name, double rad);
  double convertUnitToRad(const std::string & joint_name, int unit);

  void readJointPositions(std::map<std::string, JointState> & pos_map);
  void setJointPosition(const std::string & joint_name, int position, int time);

  bool manual_mode_enabled();
  void set_manual_mode(bool enable);

  // Members (Declaration order matches initialization order to prevent -Wreorder)
  bool inited_;
  std::atomic<bool> run_;

  std::unique_ptr<rhphumanoid_serial> drvr_;

  std::thread thread_;
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

  // Flags
  bool new_cmd_;
};

} // namespace rhphumanoid

#endif // RHPHUMANOID_HARDWARE_INTERFACE__RHPHUMANOID_HPP_
