// Copyright (C) Pololu Corporation.  See LICENSE.txt for details.

#pragma once

#include <Arduino.h>

/// The possible communication errors that can happen when reading the
/// acknowledgment packet from a servo.
enum class XYZrobotServoError
{
  /// No error.
  None = 0,

  /// There was a timeout waiting to receive the 7-byte acknowledgment header.
  HeaderTimeout = 1,

  /// The first byte of received header was not 0xFF.
  HeaderByte1Wrong = 2,

  /// The second byte of the received header was not 0xFF.
  HeaderByte2Wrong = 3,

  /// The ID byte in the received header was wrong.
  IdWrong = 4,

  /// The CMD bytes in the received header was wrong.
  CmdWrong = 5,

  /// The size byte in the received header was wrong.
  SizeWrong = 6,

  /// There was a timeout reading the first expected block of data in the
  /// acknowledgment.
  Data1Timeout = 7,

  /// There was a timeout reading the second expected block of data in the
  /// acknowledgment.
  Data2Timeout = 8,

  /// The first byte of the checksum was wrong.
  Checksum1Wrong = 9,

  /// The second byte of the checksum was wrong.
  Checksum2Wrong = 10,

  /// The offset byte returned by an EEPROM Read or RAM Read command was wrong.
  ReadOffsetWrong = 16,

  /// The length byte returned by an EEPROM Read or RAM Read command was wrong.
  ReadLengthWrong = 17,
};

/// The possible values for the ACK_Policy parameter stored in the servo's
/// EEPROM and RAM.  This parameter determins which commands the servo will send
/// an acknowledgment response for.
enum class XYZrobotServoAckPolicy
{
  // The servo only responds to STAT commands.
  OnlyStat = 0,

  // The servo only responds to STAT, EEPROM Read, and RAM Read commands.
  OnlyReadAndStat = 1,

  // The servo responds to all commands.
  All = 2,
};

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

  XYZrobotServoAckPolicy readAckPolicyRam();

  XYZrobotServoStatus readStatus();

  void setTargetPosition(uint16_t position, uint8_t playtime = 0);

  /// Returns the communication error from the last command.  The return value
  /// will be 0 if there was no error and non-zero if there was an error.  The
  /// return value will be one of the values of the XYZrobotServoError enum.
  uint8_t getLastError() const { return (uint8_t)lastError; }

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

  XYZrobotServoError lastError;
  uint8_t lastStatusError;
  uint8_t lastStatusDetail;

  uint8_t id;
  Stream * stream;
};


