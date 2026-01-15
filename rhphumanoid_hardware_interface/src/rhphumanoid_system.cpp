// Copyright 2020 ros2_control Development Team
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

#include "rhphumanoid_hardware_interface/rhphumanoid_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#undef FAKE_IT

namespace rhphumanoid_hardware
{
  // [수정] 인자 타입 변경 (HardwareInfo -> HardwareComponentInterfaceParams)
  hardware_interface::CallbackReturn RHPHumanoidSystemHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
  {
   // [수정] 부모 클래스 초기화도 params로 호출
   if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
   {
     return hardware_interface::CallbackReturn::ERROR;
   }

  // 이후 코드는 info_ 를 그대로 사용하면 됩니다. (부모 클래스가 세팅해줌)
  RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Number of joints= %ld", info_.joints.size());

  // 1. 파라미터가 존재하는지 확인하고, 없으면 기본값 0.0 또는 1.0을 사용
  if (info_.hardware_parameters.count("example_param_hw_start_duration_sec")) {
    hw_start_sec_ = stod(info_.hardware_parameters.at("example_param_hw_start_duration_sec"));
  } else {
    hw_start_sec_ = 0.0; // 기본값
  }

  if (info_.hardware_parameters.count("example_param_hw_stop_duration_sec")) {
    hw_stop_sec_ = stod(info_.hardware_parameters.at("example_param_hw_stop_duration_sec"));
  } else {
    hw_stop_sec_ = 0.0; // 기본값
  }

  if (info_.hardware_parameters.count("example_param_hw_slowdown")) {
    hw_slowdown_ = stod(info_.hardware_parameters.at("example_param_hw_slowdown"));
  } else {
    hw_slowdown_ = 10.0; // 기본값
  }

  // 2. Initial Value도 비어있을 수 있으므로 예외 처리 추가
  for(int i = 0; i < (int)info_.joints.size(); i++){
    if (!info_.joints[i].state_interfaces[0].initial_value.empty()) {
        joint_initial_value[i] = stod(info_.joints[i].state_interfaces[0].initial_value);
    } else {
        joint_initial_value[i] = 0.0; // 초기값이 없으면 0도로 설정
    }
  }

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_joint_name_.resize(info_.joints.size(), std::string());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_last_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RHPHumanoidSystemHardware"),
        "Joint '%s' has %ld command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RHPHumanoidSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RHPHumanoidSystemHardware"),
        "Joint '%s' has %ld state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RHPHumanoidSystemHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Configured");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RHPHumanoidSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
	  hw_joint_name_[i] = info_.joints[i].name;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RHPHumanoidSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RHPHumanoidSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Starting ...please wait...");

#if !defined(FAKE_IT)
  if (!rhphumanoid.init()) {
	  return hardware_interface::CallbackReturn::ERROR;
  }
#endif

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RHPHumanoidSystemHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }

  // set some default values when starting the first time
  // rad 값 input
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = hw_commands_[i] = joint_initial_value[i];
    hw_commands_last_[i] = std::numeric_limits<double>::max();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RHPHumanoidSystemHardware"), "System Successfully started!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RHPHumanoidSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Stopping ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RHPHumanoidSystemHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RHPHumanoidSystemHardware"), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

#if !defined(FAKE_IT)
hardware_interface::return_type RHPHumanoidSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  std::vector<double> positions;
  rhphumanoid.getAllJointPositions(positions, hw_joint_name_);

  for (uint i = 0; i < hw_states_.size(); i++)
  {
	  if (hw_states_[i] != positions[i])
	  {
		  hw_states_[i] = positions[i];
		  // RCLCPP_DEBUG(
		  //	  rclcpp::get_logger("RHPHumanoidSystemHardware"), "New state %.5f for joint %s (%d)",
		  //	  hw_states_[i], hw_joint_name_[i].c_str(), i);
	  }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RHPHumanoidSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  rhphumanoid.setAllJointPositions(hw_commands_, hw_joint_name_);
  return hardware_interface::return_type::OK;
}


#else
hardware_interface::return_type RHPHumanoidSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  for (uint i = 0; i < hw_states_.size(); i++)
  {
		double new_state = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
	    if ( new_state != hw_states_[i]) {
	    	hw_states_[i] = new_state;
			RCLCPP_DEBUG(
			  rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "New state %.5f for joint %d!",
			  hw_states_[i], i);
	    }
	  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RHPHumanoidSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
	  if (hw_states_[i] != hw_commands_[i]) {
		RCLCPP_INFO(
		  rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "New command %.5f for joint %d",
		  hw_commands_[i], i);
	  }
  }

  return hardware_interface::return_type::OK;
}
#endif // FAKE_IT

}  // namespace rhphumanoid_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
		rhphumanoid_hardware::RHPHumanoidSystemHardware, hardware_interface::SystemInterface)
