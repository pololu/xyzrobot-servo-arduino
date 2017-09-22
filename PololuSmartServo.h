// Copyright (C) Pololu Corporation.  See LICENSE.txt for details.

#pragma once

#include <Arduino.h>

class PololuSmartServo {
public:
  struct Status
  {
    uint8_t statusError;  // TODO: remove?
    uint8_t statusDetail; // TODO: remove?
    uint16_t pwm;
    uint16_t targetPosition;  // "pos_ref" in the A1-16 datasheet.
    uint16_t currentPosition; // "position" in the A1-16 datasheet.
    uint16_t busCurrent;      // "Ibus" in the A1-16 datasheet.
  } __attribute__((packed));

  PololuSmartServo(Stream &, uint8_t id);

  Status readStatus();

  void setTargetPosition(uint16_t position, uint8_t playtime);

  uint8_t getLastError() const { return lastError; }

  uint8_t getLastStatusError() const { return lastStatusError; }
  uint8_t getLastStatusDetail() const { return lastStatusDetail; }

private:
  void sendRequest(uint8_t cmd, const uint8_t * data, uint8_t dataSize);
  void readAck(uint8_t cmd, uint8_t * data, uint8_t dataSize);
  void sendIJog(uint16_t goal, uint8_t type, uint8_t playTime);

  uint8_t lastError;
  uint8_t lastStatusError;
  uint8_t lastStatusDetail;

  uint8_t id;
  Stream * stream;
};


