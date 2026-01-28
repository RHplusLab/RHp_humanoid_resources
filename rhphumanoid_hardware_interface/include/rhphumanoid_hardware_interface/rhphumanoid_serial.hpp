
#ifndef RHPHUMANOID_SERIAL__H
#define RHPHUMANOID_SERIAL__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <vector>

#include "rhphumanoid_drvr.hpp"

namespace rhphumanoid
{
	class rhphumanoid_serial: public rhphumanoid_drvr
	{
		public:
			rhphumanoid_serial();
			~rhphumanoid_serial();

			bool open(const std::string &portname) override;
			void close() override;

			bool getJointPosition(int id, uint16_t &pos) override;
			bool setJointPosition(int id, uint16_t pos, uint16_t time) override;
			bool setManualModeAll(bool enable, int count) override;

		private:
			int fd_;
	};
}

#endif // RHPHUMANOID_SERIAL__H
