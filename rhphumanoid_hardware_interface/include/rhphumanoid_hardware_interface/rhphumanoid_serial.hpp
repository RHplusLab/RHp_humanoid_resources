#ifndef RHPHUMANOID_SERIAL__H
#define RHPHUMANOID_SERIAL__H
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

            // [추가] 오버라이드 선언
            bool setMultiJointPositions(const std::vector<uint8_t> &ids, const std::vector<uint16_t> &positions, uint16_t time) override;

			bool setManualModeAll(bool enable, int count) override;

		private:
			int fd_;
	};
}

#endif // RHPHUMANOID_SERIAL__H
