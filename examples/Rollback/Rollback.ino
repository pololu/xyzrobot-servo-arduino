// This example shows how to send the Rollback command to reset a
// servo's settings in EEPROM to their defaults.
//
// Before using this, be sure to set the parameter below.

// Change this to be the servo's ID.
const uint8_t servoId = 1;

#include <XYZrobotServo.h>

// On boards with a hardware serial port available for use, use
// that port to communicate with the Tic. For other boards,
// create a SoftwareSerial object using pin 10 to receive (RX)
// and pin 11 to transmit (TX).
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define servoSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial servoSerial(10, 11);
#endif

XYZrobotServo servo(servoSerial, servoId);

void setup()
{
  delay(2000);
  servo.rollback();
}

void loop()
{
  // Nothing to do here.
}
