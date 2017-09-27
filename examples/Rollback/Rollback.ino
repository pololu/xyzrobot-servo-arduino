// This example shows how to send the Rollback command to reset a
// servo's settings in EEPROM to their defaults.
//
// Note that this will reset the servo's ID to 1, so you should
// NOT do this if there is already another servo on your chain
// with ID 1, since you could cause an ID conflict.
//
// Before using this, be sure to set the servoId variable below.

// Change this to be the servo's ID.
const uint8_t servoId = 1;

#include <XYZrobotServo.h>

// On boards with a hardware serial port available for use, use
// that port. For other boards, create a SoftwareSerial object
// using pin 10 to receive (RX) and pin 11 to transmit (TX).
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define servoSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial servoSerial(10, 11);
#endif

XYZrobotServo servo(servoSerial, servoId);

void setup()
{
  servoSerial.begin(115200);
  delay(2000);

  servo.rollback();
}

void loop()
{
  // Nothing to do here.
}
