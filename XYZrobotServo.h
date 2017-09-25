// Copyright (C) Pololu Corporation.  See LICENSE.txt for details.

#pragma once

#include <Arduino.h>

struct XYZrobotServoStatus
{
  uint8_t statusError;
  uint8_t statusDetail;
  uint16_t pwm;
  uint16_t posRef;
  uint16_t position;
  uint16_t iBus;
} __attribute__((packed));

class XYZrobotServo {
public:
  XYZrobotServo(Stream &, uint8_t id);

  void eepromRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize);

  // The A1-16 seems to return an invalid response if the data size is more than
  // 35.
  void ramRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize);

  XYZrobotServoStatus readStatus();

  void setTargetPosition(uint16_t position, uint8_t playtime = 0);

  uint8_t getLastError() const { return lastError; }

  uint8_t getLastStatusError() const { return lastStatusError; }
  uint8_t getLastStatusDetail() const { return lastStatusDetail; }

private:
  void flushRead();
  void sendRequest(uint8_t cmd, const uint8_t * data, uint8_t dataSize);
  void readAck(uint8_t cmd,
    uint8_t * data1, uint8_t data1Size,
    uint8_t * data2, uint8_t data2Size);
  void memoryRead(uint8_t cmd, uint8_t startAddress, uint8_t * data, uint8_t dataSize);
  void sendIJog(uint16_t goal, uint8_t type, uint8_t playTime);

  uint8_t lastError;
  uint8_t lastStatusError;
  uint8_t lastStatusDetail;

  uint8_t id;
  Stream * stream;
};


