#ifndef SERIAL_SERVO_BUS__H
#define SERIAL_SERVO_BUS__H

#include <cstdint>
#include <vector>

// Bus Servo Controller Protocol Commands
#define LOBOT_SERVO_FRAME_HEADER         0x55
#define CMD_SERVO_MOVE                   0x03
#define CMD_ACTION_GROUP_RUN             0x06
#define CMD_ACTION_GROUP_STOP            0x07
#define CMD_ACTION_GROUP_COMPLETE        0x08
#define CMD_MULT_SERVO_UNLOAD            0x14
#define CMD_MULT_SERVO_POS_READ          0x15

// Bulk Move 함수 추가 (vector 사용)
bool LobotSerialServoMoveBulk(int fd, const std::vector<uint8_t> &ids, const std::vector<uint16_t> &positions, uint16_t time);

bool LobotSerialServoMove(int fd, uint8_t id, int16_t position, uint16_t time);
bool LobotSerialServoStopMove(int fd, uint8_t id);
bool LobotSerialServoSetID(int fd, uint8_t oldID, uint8_t newID);
bool LobotSerialServoSetMode(int fd, uint8_t id, uint8_t Mode, int16_t Speed);
bool LobotSerialServoLoad(int fd, uint8_t id);
bool LobotSerialServoUnload(int fd, uint8_t id);

bool LobotSerialServoReadPosition(int fd, uint8_t id, uint16_t &position);
bool LobotSerialServoReadVin(int fd, uint8_t id, uint16_t &vin);

#endif // SERIAL_SERVO_BUS__H
