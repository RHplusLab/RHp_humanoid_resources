// Serial servo bus control for Bus Servo Controller v1.0
// Modified to match the "Bus Servo Controller Communication Protocol"

#include <iostream>
#include <cstdint>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "rhptwo_hardware_interface/serial_servo_bus.hpp"

#define GET_LOW_uint8_t(A) (uint8_t)((A))
#define GET_HIGH_uint8_t(A) (uint8_t)((A) >> 8)
#define uint8_t_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

// Write helper
static bool writeMsg(int fd, const uint8_t *buf, int len) {
  if (len != write(fd, buf, len)) {
    return false;
  }
  tcdrain(fd);
  return true;
}

// Bulk Write 구현 (여러 모터 동시 제어)
// Packet: Header(2) + Length(1) + Cmd(1) + Count(1) + Time(2) + [ID(1) + Pos(2)]...
bool LobotSerialServoMoveBulk(int fd, const std::vector<uint8_t> &ids, const std::vector<uint16_t> &positions, uint16_t time) {
  if (ids.empty() || ids.size() != positions.size()) {
    return false;
  }

  int count = ids.size();
  if (count > 255) count = 255; // 1바이트 제한

  // 패킷 길이 계산
  // Length = Cmd(1) + Count(1) + Time(2) + (ID(1) + Pos(2)) * N
  //        = 4 + 3*N
  // Total Length = Header(2) + Length(1) + Cmd(1) + Count(1) + Time(2) + ...
  // 문서상 Data Length 필드 값 = 파라미터 수 + 2 (Cmd+Len자신?) -> 아니면 그냥 파라미터+1(Cmd)?
  // 문서 2.1 "Data Length: N + 2" (N=파라미터 개수, 2=Cmd+Length자체)
  // 파라미터 구성: Count(1) + Time(2) + [ID(1)+Pos(2)]*Count
  // 파라미터 바이트 수 N = 3 + 3*Count
  // 따라서 Length 필드 값 = (3 + 3*Count) + 2 = 5 + 3*Count

  int dataLenField = 5 + (3 * count);
  int totalPacketLen = 2 + dataLenField; // Header(2) + 나머지

  std::vector<uint8_t> buf(totalPacketLen);

  buf[0] = LOBOT_SERVO_FRAME_HEADER;
  buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = dataLenField;
  buf[3] = CMD_SERVO_MOVE;
  buf[4] = count;
  buf[5] = GET_LOW_uint8_t(time);
  buf[6] = GET_HIGH_uint8_t(time);

  for (int i = 0; i < count; i++) {
    int idx = 7 + (i * 3);
    buf[idx] = ids[i];
    buf[idx+1] = GET_LOW_uint8_t(positions[i]);
    buf[idx+2] = GET_HIGH_uint8_t(positions[i]);
  }

  return writeMsg(fd, buf.data(), totalPacketLen);
}

// CMD_SERVO_MOVE (0x03) - 단일 제어
bool LobotSerialServoMove(int fd, uint8_t id, int16_t position, uint16_t time) {
  const int msgLen = 10;
  uint8_t buf[msgLen];

  if (position < 0) position = 0;
  if (position > 1000) position = 1000;

  buf[0] = LOBOT_SERVO_FRAME_HEADER;
  buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = 8;                       // Length
  buf[3] = CMD_SERVO_MOVE;          // 0x03
  buf[4] = 1;                       // Count
  buf[5] = GET_LOW_uint8_t(time);   // Time LSB
  buf[6] = GET_HIGH_uint8_t(time);  // Time MSB
  buf[7] = id;                      // Servo ID
  buf[8] = GET_LOW_uint8_t(position);// Position LSB
  buf[9] = GET_HIGH_uint8_t(position);// Position MSB

  return writeMsg(fd, buf, msgLen);
}

// CMD_MULT_SERVO_UNLOAD (0x14)
bool LobotSerialServoUnload(int fd, uint8_t id) {
  const int msgLen = 6;
  uint8_t buf[msgLen];

  buf[0] = LOBOT_SERVO_FRAME_HEADER;
  buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = 4;                       // Length
  buf[3] = CMD_MULT_SERVO_UNLOAD;   // 0x14
  buf[4] = 1;                       // Count
  buf[5] = id;                      // ID

  return writeMsg(fd, buf, msgLen);
}

bool LobotSerialServoLoad(int /*fd*/, uint8_t /*id*/) {
  return true;
}

// CMD_MULT_SERVO_POS_READ (0x15)
bool LobotSerialServoReadPosition(int fd, uint8_t id, uint16_t &position) {
  const int reqLen = 6;
  uint8_t buf[reqLen];

  buf[0] = LOBOT_SERVO_FRAME_HEADER;
  buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = 4;                       // Length
  buf[3] = CMD_MULT_SERVO_POS_READ; // 0x15
  buf[4] = 1;                       // Count
  buf[5] = id;                      // ID

  tcflush(fd, TCIFLUSH);

  if (!writeMsg(fd, buf, reqLen)) {
    return false;
  }

  uint8_t respBuf[16];
  int n = 0;
  int totalRead = 0;
  int expectedLen = 8;
  int retries = 10;

  while (totalRead < expectedLen && retries > 0) {
    n = read(fd, respBuf + totalRead, expectedLen - totalRead);
    if (n > 0) {
      totalRead += n;
    } else {
      usleep(1000); // 1ms
      retries--;
    }
  }

  if (totalRead >= expectedLen) {
    if (respBuf[0] == LOBOT_SERVO_FRAME_HEADER && respBuf[1] == LOBOT_SERVO_FRAME_HEADER) {
      if (respBuf[5] == id) {
        position = uint8_t_TO_HW(respBuf[7], respBuf[6]);
        return true;
      }
    }
  }

  return false;
}

// Dummy functions
bool LobotSerialServoStopMove(int /*fd*/, uint8_t /*id*/) { return false; }
bool LobotSerialServoSetID(int /*fd*/, uint8_t /*oldID*/, uint8_t /*newID*/) { return false; }
bool LobotSerialServoSetMode(int /*fd*/, uint8_t /*id*/, uint8_t /*Mode*/, int16_t /*Speed*/) { return false; }
bool LobotSerialServoReadVin(int /*fd*/, uint8_t /*id*/, uint16_t &/*vin*/) { return false; }
