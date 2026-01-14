#include <stdexcept>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <chrono>
#include <math.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rhptwo_hardware_interface/rhptwo.hpp"

#include "rhptwo_hardware_interface/rhptwo_serial.hpp"

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
const std::string MANUAL_MODE_ENABLE_FILE = "/tmp/rhptwo_enable_manual_mode";

const int FIRST_SET_MOVE_TIME = 1500;

const int NUM_JOINTS = 7;

const std::string SERIAL_DEV = "/dev/ttyUSB0";

namespace rhptwo
{
	rhptwo::rhptwo() : inited_(false),
						   run_(false),
						   gripper_pos_min_m_(0.0),
						   gripper_pos_min_s_(0.0),
						   gripper_pos_max_s_(0.0),
						   gripper_pos_m_to_s_factor_(0.0),
						   new_cmd_(false)
	{
	}

	rhptwo::~rhptwo()
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

	bool rhptwo::init()
	{
		if (inited_)
		{
			return false;
		}

		std::string dev;
#if defined(RHPTWO_USB)
		drvr_ = std::make_unique<rhptwo_usb>();
#else
		drvr_ = std::make_unique<rhptwo_serial>();
		dev = SERIAL_DEV;
#endif
		if (!drvr_)
		{
			return false;
		}

		if (!drvr_->open(dev))
		{
			RCLCPP_ERROR(rclcpp::get_logger("RHPTwoSystemHardware"), "Failed to open driver");
			return false;
		}

		joint_name_map_.insert(std::make_pair("joint1", 1));
		joint_name_map_.insert(std::make_pair("joint2", 2));

		// range
		// 										rad   min  max  mid   invert
		joint_range_limits_["joint1"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["joint2"] = {RAD_RANGE, 0, 1000, 500, 1};

		RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "Joint limits:");

		for (const auto &j : joint_name_map_)
		{
			const auto &name = j.first;
			last_pos_set_map_[name] = {INVALID_POS, false};
			last_pos_get_map_[name] = {INVALID_POS, false};

			// Print ranges in radians
			RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "Joint: %s,  min,max:  %f, %f",
						name.c_str(),
						jointValueToPosition(name, joint_range_limits_[name].min),
						jointValueToPosition(name, joint_range_limits_[name].max));
		}

		// Read the initial positions before starting the thread that will handle that
		// from then on
		readJointPositions(last_pos_get_map_);

		run_ = true;
		thread_ = std::thread{std::bind(&rhptwo::Process, this)};

		inited_ = true;
		return true;
	}

	// Set position of all joint positions.  Any changes to the positions will be applied on the next
	// periodic update.  Any previously specified update position that has not be applied yet will be
	// dropped.
	void rhptwo::setAllJointPositions(const std::vector<double> &commands, const std::vector<std::string> &joints)
	{
		std::lock_guard<std::mutex> guard(mutex_);
		if (std::isfinite(commands[0]) == 1)
		{
			for (uint i = 0; i < commands.size(); i++)
			{
				const std::string &name = joints[i];

				int joint_pos = positionToJointValue(name, commands[i]);
				if (joint_pos != last_pos_set_map_[name].pos)
				{
					RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "New pos cmd %*s %s: %.5f",
								i * 8, "",
								name.c_str(),
								commands[i]);
					last_pos_set_map_[name] = {joint_pos, true};
					// Run in open-loop while moving by immediately reporting the movement has completed
					// since reading the actual position from the servos during motion causes too much
					// delay and jerky motion as a result.  Once motion stops, the actual joint positions
					// will updated by the update thread.
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

	// Get position of all joints.  The returned position vector corresponds to the last periodic update.
	void rhptwo::getAllJointPositions(std::vector<double> &positions, const std::vector<std::string> &joints)
	{
		std::lock_guard<std::mutex> guard(mutex_);
		for (uint i = 0; i < joints.size(); i++)
		{
			positions.push_back(jointValueToPosition(joints[i], last_pos_get_map_[joints[i]].pos));
			RCLCPP_DEBUG(rclcpp::get_logger("RHPTwoSystemHardware"), "Get cur pos %*s %s: %.5f",
						 i * 8, "",
						 joints[i].c_str(),
						 positions[i]);
		}
	}

	// input : rad
	// output : unit(0 ~ 1000)
	int rhptwo::convertRadToUnit(std::string joint_name, double rad)
	{
		// Range in servo units
		double range = joint_range_limits_[joint_name].max - joint_range_limits_[joint_name].min;
		// Mid-range in servo units
		// double b = joint_range_limits_[joint_name].min.max - range/2;
		double b = joint_range_limits_[joint_name].mid;
		return (range * rad / joint_range_limits_[joint_name].range_rad * joint_range_limits_[joint_name].invert_factor) + b;
	}

	// input : unit(0 ~ 1000)
	// output : rad
	double rhptwo::convertUnitToRad(std::string joint_name, int unit)
	{
		// Range in servo units
		double range = joint_range_limits_[joint_name].max - joint_range_limits_[joint_name].min;
		// Mid-range in servo units
		double b = joint_range_limits_[joint_name].mid;
		// double b = joint_range_limits_[joint_name].min.max - range/2;
		return (unit - b) * joint_range_limits_[joint_name].range_rad * joint_range_limits_[joint_name].invert_factor / range;
	}

	// input : jointValue : 0 ~ 1000 모터 단위
	// output : position : moveit에서 날라오는 rad 값 (그리퍼의 경우 예외로 그리퍼 사이의 m 거리값)
	double rhptwo::jointValueToPosition(std::string joint_name, int jointValue)
	{
		double position = 0.0;
		position = convertUnitToRad(joint_name, jointValue);

		return position;
	}

	// input : Position : moveit에서 날라오는 rad 값 (그리퍼의 경우 예외로 그리퍼 사이의 m 거리값)
	// output : jointValue : 0 ~ 1000 모터 단위
	int rhptwo::positionToJointValue(std::string joint_name, double position)
	{
		int jointValue = 0;
		jointValue = int(convertRadToUnit(joint_name, position));

		return jointValue;
	}

	// Read all joint positions
	void rhptwo::readJointPositions(PositionMap &pos_map)
	{
		RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "readJointPositions start");

		int joint_id;
		for (auto const &j : joint_name_map_)
		{
			std::string name = j.first;
			joint_id = j.second;

			uint16_t p;
			if (!drvr_->getJointPosition(joint_id, p))
			{
				RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "getJointsPosition error for joint: %d", joint_id);
				continue;
			}
			pos_map[name] = {p, true};
			RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "Read servo %s, pos= %d, %f",
						name.c_str(), p, jointValueToPosition(name, p));
		}
	}

	// Set the specified joint position
	void rhptwo::setJointPosition(std::string joint_name, int position, int time)
	{
		RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "Set servo %s, pos= %d, time %d",
					joint_name.c_str(), position, time);

		if (!drvr_->setJointPosition(joint_name_map_[joint_name], position, time))
		{
			RCLCPP_ERROR(rclcpp::get_logger("RHPTwoSystemHardware"), "Failed to set joint position for servo %s",
						 joint_name.c_str());
		}
		return;
	}

	// Check for the file that is used to manually nenable/disable this mode for testing
	bool rhptwo::manual_mode_enabled()
	{
		return access(MANUAL_MODE_ENABLE_FILE.c_str(), F_OK) != -1;
	}

	// Manual mode turns off the motor in the servo so you can back drive to a desired position
	void rhptwo::set_manual_mode(bool enable)
	{
		RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "Enable manual mode: %C", enable ? 'Y' : 'N');

		if (!drvr_->setManualModeAll(enable, NUM_JOINTS))
		{
			RCLCPP_ERROR(rclcpp::get_logger("RHPTwoSystemHardware"), "Failed to set joint mode enable");
		}
	}

	void rhptwo::Process()
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

			RCLCPP_DEBUG(rclcpp::get_logger("RHPTwoSystemHardware"), "Update");

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
						// Periodically print each joint position while in manual mode
						RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "In Manual mode, joint positions:");
						for (auto const &p : last_pos_get_map_)
						{
							RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "  Pos: %d,  Joint: %s", p.second.pos, p.first.c_str());
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
						RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "Update, joint %s, pos= %d, delta= %d",
									joint.c_str(), set_pos, set_pos - pos_map[joint].pos);
						setJointPosition(joint, set_pos, first_set ? FIRST_SET_MOVE_TIME : UPDATE_PERIOD_MOVING_MS);
					}
				}
				first_set = false;

				if (idle)
				{
					RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "Entering running mode");
					idle = false;
				}
				ck_for_idle_cnt = 0;
				ck_for_manual_mode_cnt = 0;
			}
			else if (!idle && ck_for_idle_cnt++ > IDLE_ENTRY_CNT)
			{
				idle = true;
				RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "Entering idle mode");
			}

			// Don't read while moving since it causes jerks in the motion.  Update after commands stop.
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

			// Sleep for whatever remaining time until the next update
			std::this_thread::sleep_until(next_update_time);
		}
	}

}
