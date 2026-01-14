
#ifndef RHPTWO_DRVR__H
#define RHPTWO_DRVR__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <vector>

namespace rhptwo
{
	class rhptwo_drvr
	{
		public:
			rhptwo_drvr() {};
			virtual ~rhptwo_drvr() {};

			virtual bool open(const std::string &portname) = 0;
			virtual void close() = 0;

			virtual bool getJointPosition(int id, uint16_t &pos) = 0;
			virtual bool setJointPosition(int id, uint16_t pos, uint16_t time) = 0;
			virtual bool setManualModeAll(bool enable, int count) = 0;
	};
}

#endif // RHPTWO_DRVR__H
