#ifndef RHPHUMANOID_DRVR__H
#define RHPHUMANOID_DRVR__H

#include <string>
#include <cstdint>
#include <vector>

namespace rhphumanoid
{
	class rhphumanoid_drvr
	{
		public:
			virtual ~rhphumanoid_drvr() {}

			virtual bool open(const std::string &portname) = 0;
			virtual void close() = 0;

			virtual bool getJointPosition(int id, uint16_t &pos) = 0;
			virtual bool setJointPosition(int id, uint16_t pos, uint16_t time) = 0;

            // [추가] 다중 관절 제어 가상 함수
			virtual bool setMultiJointPositions(const std::vector<uint8_t> &ids, const std::vector<uint16_t> &positions, uint16_t time) = 0;

			virtual bool setManualModeAll(bool enable, int count) = 0;
	};
}
#endif // RHPHUMANOID_DRVR__H
