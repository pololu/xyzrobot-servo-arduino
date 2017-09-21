// Copyright (C) Pololu Corporation.  See LICENSE.txt for details.

#pragma once

#include <Arduino.h>

class PololuSmartServo {
public:
  struct Status
  {
    uint8_t status_error;
    uint8_t status_detail;
    uint16_t pwm;
    uint16_t pos_ref;
    uint16_t position;
    uint16_t lbus;
  };

  PololuSmartServo(Stream &, uint8_t id);

  void readStatus();

  uint8_t getLastError() { return lastError; }

private:
  void sendCmd(uint8_t cmd, const uint8_t * data, uint8_t data_size);
  void readAck(uint8_t exp_cmd, const uint8_t * data, uint8_t exp_data_size);

  uint8_t lastError;
  uint8_t id;
  Stream * stream;
};


