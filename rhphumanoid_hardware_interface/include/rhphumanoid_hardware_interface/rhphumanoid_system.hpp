#ifndef RHPHUMANOID_SYSTEM__HPP
#define RHPHUMANOID_SYSTEM__HPP

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rhphumanoid_hardware_interface/visibility_control.hpp"

#include "rhphumanoid_hardware_interface/rhphumanoid.hpp"

namespace rhphumanoid_hardware
{
class RHPHumanoidSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RHPHumanoidSystemHardware)

  // [수정] 매크로 이름을 visibility_control.hpp에 정의된 것과 일치시킴
  RHPHUMANOID_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

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
  // Parameters for the RHPHumanoid simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_commands_last_;
  std::vector<double> hw_states_;
  std::vector<std::string> hw_joint_name_;

  // 초기값 저장을 위한 map
  std::map<int, double> joint_initial_value;

  rhphumanoid::rhphumanoid rhphumanoid;
};

}  // namespace rhphumanoid_hardware

#endif  // RHPHUMANOID_SYSTEM__HPP
