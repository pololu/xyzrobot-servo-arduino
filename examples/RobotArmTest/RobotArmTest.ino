// This sketch does a basic test of the XYZrobot 6 DOF Robot Arm
// Kit:
//
//  https://www.pololu.com/product/2743
//
// It sends the arm to a starting position.  Then it moves each
// of the servos by a small amount, one at a time, in order by
// servo ID.

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

XYZrobotServo servo1(servoSerial, 1);
XYZrobotServo servo2(servoSerial, 2);
XYZrobotServo servo3(servoSerial, 3);
XYZrobotServo servo4(servoSerial, 4);
XYZrobotServo servo5(servoSerial, 5);
XYZrobotServo servo6(servoSerial, 6);

// The time, in units of 10 ms, that movements should take to
// complete.
const uint8_t playtime = 75;

void startingPosition()
{
  servo1.setPosition(513, playtime);
  servo2.setPosition(403, playtime);
  servo3.setPosition(479, playtime);
  servo4.setPosition(405, playtime);
  servo5.setPosition(222, playtime);
  servo6.setPosition(600, playtime);
}

void setup()
{
  servoSerial.begin(115200);

  servoSerial.setTimeout(10);
}

void loop()
{
  delay(2000);
  startingPosition();
  delay(3000);
  servo1.setPosition(563, playtime);
  delay(1000);
  servo2.setPosition(453, playtime);
  delay(1000);
  servo3.setPosition(529, playtime);
  delay(1000);
  servo4.setPosition(455, playtime);
  delay(1000);
  servo5.setPosition(272, playtime);
  delay(1000);
  servo6.setPosition(655, playtime);
  delay(1000);
}
