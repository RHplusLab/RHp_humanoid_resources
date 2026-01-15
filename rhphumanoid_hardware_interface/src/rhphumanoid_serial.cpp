// Serial servo bus control for LewanSoul/Hiwonder servos

#include <stdexcept>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <chrono>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rhphumanoid_hardware_interface/rhphumanoid_serial.hpp"
#include "rhphumanoid_hardware_interface/serial_servo_bus.hpp"

static int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static void set_mincount(int fd, int mcount, int to_x100ms)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        fprintf(stderr, "Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = to_x100ms;

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        fprintf(stderr, "Error tcsetattr: %s\n", strerror(errno));
}

namespace rhphumanoid
{
	rhphumanoid_serial::rhphumanoid_serial():
		rhphumanoid_drvr(),
		fd_(-1)
	{
	}

	rhphumanoid_serial::~rhphumanoid_serial()
	{
		close();
	}

	bool rhphumanoid_serial::open(const std::string &portname)
	{
		if (fd_ > 0) {
			return false;
		}

		fd_ = ::open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    	if (fd_ < 0) {
			RCLCPP_ERROR(rclcpp::get_logger("RHPHumanoidSystemHardware"), "rhpHumanoid, unable to open serial port [%s]",
				strerror(errno));
			return false;
		}

  		// Baud rate 9600 for Bus Servo Controller v1.0
	    set_interface_attribs(fd_, B9600);
		set_mincount(fd_, 0, 5);
		RCLCPP_INFO(rclcpp::get_logger("RHPHumanoidSystemHardware"), "rhpHumanoid device opened at 9600 baud");
		return true;
	}

	void rhphumanoid_serial::close()
	{
		if (fd_ > 0) {
			::close(fd_);
			fd_ = -1;
		}
	}

	bool rhphumanoid_serial::getJointPosition(int id, uint16_t &pos)
	{
		// RCLCPP_DEBUG(rclcpp::get_logger("RHPHumanoidSystemHardware"), "readJointPosition");
		if (!LobotSerialServoReadPosition(fd_, id, pos)) {
			// RCLCPP_ERROR(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Failed to read servo %d position", id);
			return false;
		}
		return true;
	}

	bool rhphumanoid_serial::setJointPosition(int id, uint16_t pos, uint16_t time)
	{
		if (!LobotSerialServoMove(fd_, id, pos, time)) {
			RCLCPP_ERROR(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Failed to set servo %u position", id);
			return false;
		}
		return true;
	}

    // [추가] Bulk Move 호출
    bool rhphumanoid_serial::setMultiJointPositions(const std::vector<uint8_t> &ids, const std::vector<uint16_t> &positions, uint16_t time)
    {
        return LobotSerialServoMoveBulk(fd_, ids, positions, time);
    }

	bool rhphumanoid_serial::setManualModeAll(bool enable, int count)
	{
		bool bOk = false;
		for (int i = 0; i < count; i++) {
			if (enable) {
				bOk = LobotSerialServoUnload(fd_, i + 1);
			} else {
				bOk = LobotSerialServoLoad(fd_, i + 1);
			}
			if (!bOk) {
				RCLCPP_ERROR(rclcpp::get_logger("RHPHumanoidSystemHardware"), "Failed to set enable mode on servo %d", i + 1);
				bOk = false;
			}
		}
		return bOk;
	}
}
