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

  /// Writes data from the specified buffer to the servo's EEPROM.
  ///
  /// After running this command, we recommend delaying for 10 ms per data byte
  /// before sending the next command to this servo, since writing to EEPROM
  /// takes some time and the servo cannot receive more commands until it is
  /// done.
  void eepromWrite(uint8_t startAddress, const uint8_t *, uint8_t dataSize);

  /// Reads data from the servo's EEPROM and stores it in the specified buffer.
  ///
  /// The data size should be 35 or less: otherwise the A1-16 seems to return a
  /// response with an invalid CRC.
  void eepromRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize);

  /// Writes data from the specified buffer to the servo's RAM.
  void ramWrite(uint8_t startAddress, const uint8_t *, uint8_t dataSize);

  /// Reads data from the servo's RAM and stores it in the specified buffer.
  ///
  /// The data size should be 35 or less: otherwise the A1-16 seems to return a
  /// response with an invalid CRC.
  void ramRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize);

  /// Write the sID parameter byte in EEPROM, which determines which ID the
  /// servo uses on its serial interface.
  ///
  /// After running this command, we recommend delaying for 10 ms before sending
  /// the next command to this servo, since writing to EEPROM takes some time
  /// and the servo cannot receive more commands until it is done.
  void writeIdEeprom(uint8_t id);

  /// Reads the sID parameter byte in EEPROM, which determines which ID the
  /// servo uses on its serial interface.
  uint8_t readIdEeprom();

  /// Write the sID parameter byte in RAM, which determines which ID the
  /// servo uses on its serial interface.
  ///
  /// Write the ACK_Policy parameter byte in RAM.
  void writeIdRam(uint8_t id);

  /// Write the ACK_Policy parameter byte in EEPROM.
  ///
  /// After running this command, we recommend delaying for 10 ms before sending
  /// the next command to this servo, since writing to EEPROM takes some time
  /// and the servo cannot receive more commands until it is done.
  void writeAckPolicyEeprom(XYZrobotServoAckPolicy);

  /// Read the ACK_Policy parameter byte in EEPROM.
  XYZrobotServoAckPolicy readAckPolicyEeprom();

  /// Write the ACK_Policy parameter byte in RAM.
  void writeAckPolicyRam(XYZrobotServoAckPolicy);

  /// Write the Alarm_LED_Policy byte in RAM.  This controls which LEDs on the
  /// servo are controlled by the user and which are controlled by the system.
  ///
  /// A 0 bit means the LED is controlled by the system, and a 1 bit means the
  /// LED is controlled by the user.
  ///
  /// - Bit 0: White LED
  /// - Bit 1: Blue LED
  /// - Bit 2: Green LED
  /// - Bit 3: Red LED
  ///
  /// To control user LEDs, see writeLedControl().
  void writeAlarmLedPolicyRam(uint8_t);

  /// After calling writeAlarmLedPolicyRam(), you can use this to control any
  /// LEDs that are configured as user LED.
  ///
  /// - Bit 0: White LED
  /// - Bit 1: Blue LED
  /// - Bit 2: Green LED
  /// - Bit 3: Red LED
  void writeLedControl(uint8_t);

  /// Read the ACK_Policy parameter byte in RAM.
  XYZrobotServoAckPolicy readAckPolicyRam();

  XYZrobotServoStatus readStatus();

  void setTargetPosition(uint16_t position, uint8_t playtime = 0);

  // Resets all parameters in EEPROM to their default values.
  //
  // After running this command, we recommend delaying for 2500 ms before
  // sending the next command to this servo, since it takes the servo a while to
  // change its parameters.
  void rollback();

  // Resets the servo.
  //
  // After running this command, we recommend delaying for 2500 ms before
  // sending the next command to this servo, since it takes the servo a while to
  // restart.
  void reboot();

  /// Returns the communication error from the last command.  The return value
  /// will be 0 if there was no error and non-zero if there was an error.  The
  /// return value will be one of the values of the XYZrobotServoError enum.
  uint8_t getLastError() const { return (uint8_t)lastError; }

  uint8_t getLastStatusError() const { return lastStatusError; }

  uint8_t getLastStatusDetail() const { return lastStatusDetail; }

private:
  void flushRead();

  void sendRequest(uint8_t cmd,
    const uint8_t * data1, uint8_t data1Size,
    const uint8_t * data2 = NULL, uint8_t data2Size = 0);

  void readAck(uint8_t cmd,
    uint8_t * data1, uint8_t data1Size,
    uint8_t * data2 = NULL, uint8_t data2Size = 0);

  void memoryWrite(uint8_t cmd, uint8_t startAddress, const uint8_t * data, uint8_t dataSize);

  void memoryRead(uint8_t cmd, uint8_t startAddress, uint8_t * data, uint8_t dataSize);

  void sendIJog(uint16_t goal, uint8_t type, uint8_t playTime);

  XYZrobotServoError lastError;
  uint8_t lastStatusError;
  uint8_t lastStatusDetail;

  uint8_t id;

  Stream * stream;
};


