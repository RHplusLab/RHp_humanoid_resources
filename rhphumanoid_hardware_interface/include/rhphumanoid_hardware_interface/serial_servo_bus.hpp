
#ifndef SERIAL_SERVO_BUS__H
#define SERIAL_SERVO_BUS__H

#include <cstdint>
#include <utility>
#include <vector>

bool LobotSerialServoMove(int fd, uint8_t id, int16_t position, uint16_t time);
bool LobotSerialServoStopMove(int fd, uint8_t id);
bool LobotSerialServoSetID(int fd, uint8_t oldID, uint8_t newID);
bool LobotSerialServoSetMode(int fd, uint8_t id, uint8_t Mode, int16_t Speed);
bool LobotSerialServoLoad(int fd, uint8_t id);
bool LobotSerialServoUnload(int fd, uint8_t id);

bool LobotSerialServoReadPosition(int fd, uint8_t id, uint16_t &position);
bool LobotSerialServoReadVin(int fd, uint8_t id, uint16_t &vin);

// Multi-bus driver helpers (2026-11):
// all MOVE_TIME_WRITE packets in one write() call
bool LobotSerialServoMoveMany(int fd, const std::vector<std::pair<uint8_t, int16_t>> &items, uint16_t time);
// position read with a timeout in ms (poll), chunked reads, signed result (a servo can report slightly below 0),
// echo-safe: only a reply frame with the same ID and command and a 2-byte payload is accepted
bool LobotSerialServoReadPositionTimeout(int fd, uint8_t id, int16_t &position, int timeout_ms);

#endif // SERIAL_SERVO_BUS__H
