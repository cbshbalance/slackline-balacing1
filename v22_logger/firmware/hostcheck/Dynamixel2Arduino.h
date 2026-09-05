#pragma once
#include "Arduino.h"
namespace ControlTableItem { enum { RETURN_DELAY_TIME, PROFILE_VELOCITY, PROFILE_ACCELERATION, CURRENT_LIMIT, HARDWARE_ERROR_STATUS, PRESENT_INPUT_VOLTAGE, PRESENT_CURRENT, PRESENT_POSITION, GOAL_POSITION, TORQUE_ENABLE }; }
enum { OP_EXTENDED_POSITION = 4, OP_POSITION = 3, OP_CURRENT = 0 };
struct Dynamixel2Arduino {
  Dynamixel2Arduino(HardwareSerial&, int);
  void setPortProtocolVersion(float); void begin(unsigned long); bool ping(uint8_t);
  bool torqueOn(uint8_t); bool torqueOff(uint8_t); bool setOperatingMode(uint8_t, uint8_t);
  bool writeControlTableItem(uint8_t item, uint8_t id, int32_t data, uint32_t timeout = 10);
  int32_t readControlTableItem(uint8_t item, uint8_t id, uint32_t timeout = 10);
  float getPresentPosition(uint8_t id, uint8_t unit = 0); bool setGoalPosition(uint8_t id, float value, uint8_t unit = 0);
  bool getTorqueEnableStat(uint8_t id);
};
