// This sketch shows how to set the speed of a servo using open
// loop speed control.
//
// Speeds are represented as numbers between -1023 and 1023.
// Setting the speed to 0 results in abrupt deceleration.
//
// This sketch only writes data to the servos; it does not
// receive anything.

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
// WARNING: Only change the ID number below to a servo that can
// rotate freely without damaging anything.
XYZrobotServo servo(servoSerial, 128);

void setup()
{
  // Turn on the serial port and set its baud rate.
  servoSerial.begin(115200);
}

void loop()
{
  delay(2500);

  // Move the servo output counter-clockwise for some time,
  // ramping up to the specified speed.
  servo.setSpeed(400);
  delay(2000);

  // Set the speed to 0 to make the servo stop abruptly.
  servo.setSpeed(0);
  delay(1000);

  // Move the servo output clockwise for some time, ramping up to
  // the specified speed.
  servo.setSpeed(-400);
  delay(1000);

  // Set the speed to -1 to make the servo smoothly ramp down to
  // a speed that is effectively zero.
  servo.setSpeed(-1);
}
