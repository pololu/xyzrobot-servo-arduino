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
// what ID number to use.  Change the number below to match the
// ID of your servo.
XYZrobotServo servo(servoSerial, 6);  // TODO: ID=1

void setup()
{
  servoSerial.begin(115200);

  servoSerial.setTimeout(10);
}

void loop()
{
  delay(2000);

  // Make sure the speed ramps up starting from 0 instead of
  // starting from a previously-remembered speed.
  servo.setSpeed(0);

  // Move the servo output counter-clockwise for some time, ramping up
  // to the specified speed.
  servo.setSpeed(400);
  delay(500);

  // TODO: turn it clockwise too

  // Turn off the motor, letting the servo smoothly stop.  You
  // could use "servo.setSpeed(0)" to stop here, but it would
  // result in an abrupt stop.
  servo.torqueOff();
}
