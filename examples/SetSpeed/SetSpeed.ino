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
  // Turn on the serial port and set its baud rate.
  servoSerial.begin(115200);

  // You can uncomment either of the lines below if you want to
  // specify which speed control mode to use.  The servo uses
  // open loop control by default.
  // servo.writeSpdctrlPolicyRam(XYZrobotServoSpdctrlPolicy::OpenLoop);
  // servo.writeSpdctrlPolicyRam(XYZrobotServoSpdctrlPolicy::CloseLoop);
}

void loop()
{
  delay(2500);

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
