// This example shows how to send the Rollback command to reset a
// servo's settings in EEPROM to their defaults.

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

// Set up a servo object, specifying what serial port to use and
// what ID number to use.
//
// WARNING: If you change the ID number below to something other
// than 1, make sure there are no other servos in your system
// with ID 1.  Otherwise, you could cause an ID conflict, because
// the servo you roll back will change its ID to 1.
XYZrobotServo servo(servoSerial, 1);

void setup()
{
  // Turn on the serial port and set its baud rate.
  servoSerial.begin(115200);

  delay(2500);

  servo.rollback();
}

void loop()
{
  // Nothing to do here.
}
