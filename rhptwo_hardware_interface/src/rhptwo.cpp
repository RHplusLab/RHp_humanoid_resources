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
#include "rhptwo_hardware_interface/rhptwo.hpp"

#include "rhptwo_hardware_interface/rhptwo_serial.hpp"

#define MAX_STR 255
#define INVALID_POS 99999 // Invalid servo value

const float RAD_RANGE = (240.0 / 180.0) * M_PI;
const int UPDATE_PERIOD_MOVING_MS = 10; // (1000ms/100Hz) = 10ms
const int UPDATE_PERIOD_IDLE_MS = 100;
const int IDLE_ENTRY_CNT = 50;

const int UPDATE_CNT_CHK_FOR_MANUAL_MODE = (2000 / UPDATE_PERIOD_IDLE_MS);
const std::string MANUAL_MODE_ENABLE_FILE = "/tmp/rhptwo_enable_manual_mode";

const int FIRST_SET_MOVE_TIME = 1500;
const int NUM_JOINTS = 7;

// [수정] 고정된 USB 장치 이름 사용
const std::string SERIAL_DEV = "/dev/ttyRHP";

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
        // 여기에 22축 모터 매핑을 추가하면 됩니다.

		// range
		joint_range_limits_["joint1"] = {RAD_RANGE, 0, 1000, 500, 1};
		joint_range_limits_["joint2"] = {RAD_RANGE, 0, 1000, 500, 1};

		RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "Joint limits:");

		for (const auto &j : joint_name_map_)
		{
			const auto &name = j.first;
			last_pos_set_map_[name] = {INVALID_POS, false};
			last_pos_get_map_[name] = {INVALID_POS, false};

			RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "Joint: %s,  min,max:  %f, %f",
						name.c_str(),
						jointValueToPosition(name, joint_range_limits_[name].min),
						jointValueToPosition(name, joint_range_limits_[name].max));
		}

		readJointPositions(last_pos_get_map_);

		run_ = true;
		thread_ = std::thread{std::bind(&rhptwo::Process, this)};

		inited_ = true;
		return true;
	}

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
					last_pos_set_map_[name] = {joint_pos, true};
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

	void rhptwo::getAllJointPositions(std::vector<double> &positions, const std::vector<std::string> &joints)
	{
		std::lock_guard<std::mutex> guard(mutex_);
		for (uint i = 0; i < joints.size(); i++)
		{
			positions.push_back(jointValueToPosition(joints[i], last_pos_get_map_[joints[i]].pos));
		}
	}

	int rhptwo::convertRadToUnit(std::string joint_name, double rad)
	{
		double range = joint_range_limits_[joint_name].max - joint_range_limits_[joint_name].min;
		double b = joint_range_limits_[joint_name].mid;
		return (range * rad / joint_range_limits_[joint_name].range_rad * joint_range_limits_[joint_name].invert_factor) + b;
	}

	double rhptwo::convertUnitToRad(std::string joint_name, int unit)
	{
		double range = joint_range_limits_[joint_name].max - joint_range_limits_[joint_name].min;
		double b = joint_range_limits_[joint_name].mid;
		return (unit - b) * joint_range_limits_[joint_name].range_rad * joint_range_limits_[joint_name].invert_factor / range;
	}

	double rhptwo::jointValueToPosition(std::string joint_name, int jointValue)
	{
		return convertUnitToRad(joint_name, jointValue);
	}

	int rhptwo::positionToJointValue(std::string joint_name, double position)
	{
		return int(convertRadToUnit(joint_name, position));
	}

	void rhptwo::readJointPositions(PositionMap &pos_map)
	{
		// RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "readJointPositions start");

		int joint_id;
		for (auto const &j : joint_name_map_)
		{
			std::string name = j.first;
			joint_id = j.second;

			uint16_t p;
			if (!drvr_->getJointPosition(joint_id, p))
			{
				// RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "getJointsPosition error for joint: %d", joint_id);
				continue;
			}
			pos_map[name] = {p, true};
		}
	}

	void rhptwo::setJointPosition(std::string joint_name, int position, int time)
	{
		drvr_->setJointPosition(joint_name_map_[joint_name], position, time);
	}

	bool rhptwo::manual_mode_enabled()
	{
		return access(MANUAL_MODE_ENABLE_FILE.c_str(), F_OK) != -1;
	}

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

			// RCLCPP_DEBUG(rclcpp::get_logger("RHPTwoSystemHardware"), "Update");

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
						RCLCPP_INFO(rclcpp::get_logger("RHPTwoSystemHardware"), "In Manual mode");
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

                // [최적화 적용] Bulk Write 전송
                std::vector<uint8_t> target_ids;
                std::vector<uint16_t> target_positions;

				for (auto const &c : cmd)
				{
                    // 모든 관절 명령 수집 (필요 시 c.second.changed 체크 가능)
                    const std::string &joint_name = c.first;
                    int id = joint_name_map_[joint_name];
                    int pos = c.second.pos;

                    target_ids.push_back((uint8_t)id);
                    target_positions.push_back((uint16_t)pos);
				}

                int move_time = first_set ? FIRST_SET_MOVE_TIME : UPDATE_PERIOD_MOVING_MS;

                if (!target_ids.empty()) {
                    drvr_->setMultiJointPositions(target_ids, target_positions, move_time);
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
}
