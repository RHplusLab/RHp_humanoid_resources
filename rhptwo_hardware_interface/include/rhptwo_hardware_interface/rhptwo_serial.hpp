
#ifndef RHPTWO_SERIAL__H
#define RHPTWO_SERIAL__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <vector>

#include "rhptwo_drvr.hpp"

namespace rhptwo
{
	class rhptwo_serial: public rhptwo_drvr
	{
		public:
			rhptwo_serial();
			~rhptwo_serial();

			bool open(const std::string &portname) override;
			void close() override;

			bool getJointPosition(int id, uint16_t &pos) override;
			bool setJointPosition(int id, uint16_t pos, uint16_t time) override;
			bool setManualModeAll(bool enable, int count) override;

		private:
			int fd_;
	};
}

#endif // RHPTWO_SERIAL__H
