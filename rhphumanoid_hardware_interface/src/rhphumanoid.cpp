#include <stdexcept>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <chrono>
#include <math.h>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rhphumanoid_hardware_interface/rhphumanoid.hpp"
#include "rhphumanoid_hardware_interface/rhphumanoid_serial.hpp"

#define MAX_STR 255
#define INVALID_POS 99999 // Invalid servo value

const float RAD_RANGE = (240.0 / 180.0) * M_PI;
const int UPDATE_PERIOD_MOVING_MS = 10; // (1000ms/100Hz) = 10ms
const int UPDATE_PERIOD_IDLE_MS = 100;

// Use the idle update period if this many 'moving' update
// periods occur without getting a move command.
const int IDLE_ENTRY_CNT = 50;

// How often to check for the file that indicates the control loop should be
// in manual mode where the user can manually move the robot arm (for specifying
// positions for training.)
const int UPDATE_CNT_CHK_FOR_MANUAL_MODE = (2000 / UPDATE_PERIOD_IDLE_MS);
// File to create to enable the manual mode
const std::string MANUAL_MODE_ENABLE_FILE = "/tmp/rhphumanoid_enable_manual_mode";

const int FIRST_SET_MOVE_TIME = 1500;

// 22 DOF
const int NUM_JOINTS = 22;

// Fixed serial device
const std::string SERIAL_DEV = "/dev/ttyRHP";

namespace rhphumanoid
{
	rhphumanoid::rhphumanoid() : inited_(false),
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
		if (inited_)
		{
			run_ = false;
			thread_.join();
		}

		if (drvr_)
		{
			drvr_->close();
		}
	}

	bool rhphumanoid::init()
	{
		if (inited_)
		{
			return false;
		}

		std::string dev;
#if defined(RHPHUMANOID_USB)
		drvr_ = std::make_unique<rhphumanoid_usb>();
#else
		drvr_ = std::make_unique<rhphumanoid_serial>();
		dev = SERIAL_DEV;
#endif
		if (!drvr_)
		{
			return false;
		}

		if (!drvr_->open(dev))
		{
			RCLCPP_ERROR(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Failed to open driver");
			return false;
		}

		// ===== Joint name -> ID mapping (24 DOF) =====
		joint_name_map_.insert(std::make_pair("l_hip_yaw", 1));
		joint_name_map_.insert(std::make_pair("l_hip_roll", 2));
		joint_name_map_.insert(std::make_pair("l_hip_pitch", 3));
		joint_name_map_.insert(std::make_pair("l_knee", 4));
		joint_name_map_.insert(std::make_pair("l_ank_pitch", 5));
		joint_name_map_.insert(std::make_pair("l_ank_roll", 6));

		joint_name_map_.insert(std::make_pair("r_hip_yaw", 7));
		joint_name_map_.insert(std::make_pair("r_hip_roll", 8));
		joint_name_map_.insert(std::make_pair("r_hip_pitch", 9));
		joint_name_map_.insert(std::make_pair("r_knee", 10));
		joint_name_map_.insert(std::make_pair("r_ank_pitch", 11));
		joint_name_map_.insert(std::make_pair("r_ank_roll", 12));

		joint_name_map_.insert(std::make_pair("l_sho_pitch", 13));
		joint_name_map_.insert(std::make_pair("l_sho_roll", 14));
		joint_name_map_.insert(std::make_pair("l_el", 15));
		joint_name_map_.insert(std::make_pair("l_wst", 16));
		joint_name_map_.insert(std::make_pair("l_grp", 17));

		joint_name_map_.insert(std::make_pair("r_sho_pitch", 18));
		joint_name_map_.insert(std::make_pair("r_sho_roll", 19));
		joint_name_map_.insert(std::make_pair("r_el", 20));
		joint_name_map_.insert(std::make_pair("r_wst", 21));
		joint_name_map_.insert(std::make_pair("r_grp", 22));

		//joint_name_map_.insert(std::make_pair("head_pan", 23));
		//joint_name_map_.insert(std::make_pair("head_tilt", 24));

		// ===== Joint limits =====
		//                          range_rad  min  max  mid  invert
		joint_range_limits_["l_sho_pitch"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["l_sho_roll"]  = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["l_el"]        = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["l_wst"]       = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["l_grp"]       = {RAD_RANGE, 0, 1000, 500, 1};

		joint_range_limits_["r_sho_pitch"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["r_sho_roll"]  = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["r_el"]        = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["r_wst"]       = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["r_grp"]       = {RAD_RANGE, 0, 1000, 500, 1};

		joint_range_limits_["l_hip_yaw"]   = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["l_hip_roll"]  = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["l_hip_pitch"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["l_knee"]      = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["l_ank_pitch"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["l_ank_roll"]  = {RAD_RANGE, 0, 1000, 500, 1};

		joint_range_limits_["r_hip_yaw"]   = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["r_hip_roll"]  = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["r_hip_pitch"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["r_knee"]      = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["r_ank_pitch"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["r_ank_roll"]  = {RAD_RANGE, 0, 1000, 500, 1};

		joint_range_limits_["head_pan"]  = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["head_tilt"] = {RAD_RANGE, 0, 1000, 500, 1};

		RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Joint limits:");
		for (const auto &j : joint_name_map_)
		{
			const auto &name = j.first;
			last_pos_set_map_[name] = {INVALID_POS, false};
			last_pos_get_map_[name] = {INVALID_POS, false};

			RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"),
						"Joint: %s,  min,max:  %f, %f",
						name.c_str(),
						jointValueToPosition(name, joint_range_limits_[name].min),
						jointValueToPosition(name, joint_range_limits_[name].max));
		}

		// Read the initial positions before starting the thread
		readJointPositions(last_pos_get_map_);

		// Diagnose servo state and send LOAD command
		for (auto const &j : joint_name_map_) {
			int joint_id = j.second;
			std::string name = j.first;

			uint16_t load_state = 0;
			if (drvr_->getJointLoadState(joint_id, load_state)) {
				RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"),
					"Servo %s (ID %d): load_state=%d", name.c_str(), joint_id, load_state);
			} else {
				RCLCPP_WARN(rclcpp::get_logger("RHPHumanoidSystemHardware"),
					"Servo %s (ID %d): failed to read load state", name.c_str(), joint_id);
			}

			uint16_t error = 0;
			if (drvr_->getJointLedError(joint_id, error)) {
				RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"),
					"Servo %s (ID %d): error_flags=%d", name.c_str(), joint_id, error);
			}

			// Send explicit LOAD command
			if (!drvr_->loadJoint(joint_id)) {
				RCLCPP_WARN(rclcpp::get_logger("RHPHumanoidSystemHardware"),
					"Servo %s (ID %d): LOAD command failed", name.c_str(), joint_id);
			}
			usleep(1000); // 1ms inter-command gap
		}

		run_ = true;
		thread_ = std::thread{std::bind(&rhphumanoid::Process, this)};

		inited_ = true;
		return true;
	}

	void rhphumanoid::setAllJointPositions(const std::vector<double> &commands, const std::vector<std::string> &joints)
	{
		std::lock_guard<std::mutex> guard(mutex_);
		if (std::isfinite(commands[0]) == 1)
		{
			for (uint i = 0; i < commands.size(); i++)
			{
				const std::string &name = joints[i];

				// Skip joints not mapped to physical servos
				if (joint_name_map_.find(name) == joint_name_map_.end()) {
					continue;
				}

				int joint_pos = positionToJointValue(name, commands[i]);
				if (joint_pos != last_pos_set_map_[name].pos)
				{
					RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "New pos cmd %*s %s: %.5f",
								i * 8, "",
								name.c_str(),
								commands[i]);

					last_pos_set_map_[name] = {joint_pos, true};

					// open-loop during motion
					last_pos_get_map_[name] = {joint_pos, false};

					new_cmd_ = true;
				}
			}
		}
		else
		{
			new_cmd_ = false;
		}
	}

	void rhphumanoid::getAllJointPositions(std::vector<double> &positions, const std::vector<std::string> &joints)
	{
		std::lock_guard<std::mutex> guard(mutex_);
		for (uint i = 0; i < joints.size(); i++)
		{
			// Return 0.0 for joints not mapped to physical servos
			if (joint_name_map_.find(joints[i]) == joint_name_map_.end()) {
				positions.push_back(0.0);
				continue;
			}
			positions.push_back(jointValueToPosition(joints[i], last_pos_get_map_[joints[i]].pos));
			RCLCPP_DEBUG(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Get cur pos %*s %s: %.5f",
						 i * 8, "",
						 joints[i].c_str(),
						 positions[i]);
		}
	}

	int rhphumanoid::convertRadToUnit(std::string joint_name, double rad)
	{
		double range = joint_range_limits_[joint_name].max - joint_range_limits_[joint_name].min;
		double b = joint_range_limits_[joint_name].mid;
		return (range * rad / joint_range_limits_[joint_name].range_rad * joint_range_limits_[joint_name].invert_factor) + b;
	}

	double rhphumanoid::convertUnitToRad(std::string joint_name, int unit)
	{
		double range = joint_range_limits_[joint_name].max - joint_range_limits_[joint_name].min;
		double b = joint_range_limits_[joint_name].mid;
		return (unit - b) * joint_range_limits_[joint_name].range_rad * joint_range_limits_[joint_name].invert_factor / range;
	}

	double rhphumanoid::jointValueToPosition(std::string joint_name, int jointValue)
	{
		return convertUnitToRad(joint_name, jointValue);
	}

	int rhphumanoid::positionToJointValue(std::string joint_name, double position)
	{
		return int(convertRadToUnit(joint_name, position));
	}

	void rhphumanoid::readJointPositions(PositionMap &pos_map)
	{
		RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "readJointPositions start");

		int joint_id;
		for (auto const &j : joint_name_map_)
		{
			std::string name = j.first;
			joint_id = j.second;

			uint16_t p;
			if (!drvr_->getJointPosition(joint_id, p))
			{
				RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "getJointsPosition error for joint: %d", joint_id);
				continue;
			}
			pos_map[name] = {p, true};

			RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Read servo %s, pos= %d, %f",
						name.c_str(), p, jointValueToPosition(name, p));
		}
	}

	void rhphumanoid::setJointPosition(std::string joint_name, int position, int time)
	{
		RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Set servo %s, pos= %d, time %d",
					joint_name.c_str(), position, time);

		if (!drvr_->setJointPosition(joint_name_map_[joint_name], position, time))
		{
			RCLCPP_ERROR(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Failed to set joint position for servo %s",
						 joint_name.c_str());
		}
		return;
	}

	bool rhphumanoid::manual_mode_enabled()
	{
		return access(MANUAL_MODE_ENABLE_FILE.c_str(), F_OK) != -1;
	}

	void rhphumanoid::set_manual_mode(bool enable)
	{
		RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Enable manual mode: %C", enable ? 'Y' : 'N');

		if (!drvr_->setManualModeAll(enable, NUM_JOINTS))
		{
			RCLCPP_ERROR(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Failed to set joint mode enable");
		}
	}

	void rhphumanoid::Process()
	{
		int read_pos_delay_cnt = 0;
		int ck_for_manual_mode_cnt = 0;
		bool manual_mode = false;
		bool idle = false;
		int ck_for_idle_cnt = 0;
		PositionMap pos_map;
		bool first_set = true;

		while (run_)
		{
			auto next_update_time = std::chrono::steady_clock::now();

			RCLCPP_DEBUG(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Update");

			if (idle && --ck_for_manual_mode_cnt <= 0)
			{
				ck_for_manual_mode_cnt = UPDATE_CNT_CHK_FOR_MANUAL_MODE;
				bool enabled = manual_mode_enabled();

				if (manual_mode)
				{
					if (!enabled)
					{
						set_manual_mode(false);
						manual_mode = false;
					}
					else
					{
						RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "In Manual mode, joint positions:");
						for (auto const &p : last_pos_get_map_)
						{
							RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"),
										"  Pos: %d,  Joint: %s", p.second.pos, p.first.c_str());
						}
					}
				}
				else if (!manual_mode && enabled)
				{
					set_manual_mode(true);
					manual_mode = true;
				}
			}

			bool new_cmd = false;
			PositionMap cmd;
			{
				std::lock_guard<std::mutex> guard(mutex_);
				if (new_cmd_)
				{
					cmd = last_pos_set_map_;
					new_cmd = true;
					new_cmd_ = false;

					for (auto &lp : last_pos_set_map_)
					{
						lp.second.changed = false;
					}
				}
			}

			if (new_cmd)
			{
				read_pos_delay_cnt = 1;

				for (auto const &c : cmd)
				{
					if (c.second.changed)
					{
						int set_pos = c.second.pos;
						const std::string &joint = c.first;

						RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Update, joint %s, pos= %d, delta= %d",
									joint.c_str(), set_pos, set_pos - pos_map[joint].pos);

						setJointPosition(joint, set_pos, first_set ? FIRST_SET_MOVE_TIME : UPDATE_PERIOD_MOVING_MS);
						usleep(1000); // 1ms gap between consecutive write commands
					}
				}
				first_set = false;

				if (idle)
				{
					RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Entering running mode");
					idle = false;
				}
				ck_for_idle_cnt = 0;
				ck_for_manual_mode_cnt = 0;
			}
			else if (!idle && ck_for_idle_cnt++ > IDLE_ENTRY_CNT)
			{
				idle = true;
				RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Entering idle mode");
			}

			// Don't read while moving since it causes jerks in the motion. Update after commands stop.
			if (!new_cmd && --read_pos_delay_cnt <= 0)
			{
				read_pos_delay_cnt = 5;
				{
					std::lock_guard<std::mutex> guard(mutex_);
					pos_map = last_pos_get_map_;
				}
				readJointPositions(pos_map);
				{
					std::lock_guard<std::mutex> guard(mutex_);
					last_pos_get_map_ = pos_map;
				}
			}

			next_update_time += std::chrono::milliseconds(idle ? UPDATE_PERIOD_IDLE_MS : UPDATE_PERIOD_MOVING_MS);
			std::this_thread::sleep_until(next_update_time);
		}
	}

} // namespace rhphumanoid
