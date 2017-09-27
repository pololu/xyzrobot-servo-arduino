// This sketch does a basic test of the XYZrobot 6 DOF Robot Arm
// Kit.

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
const uint8_t playTime = 50;

void neutralPosition()
{
  servo1.setPosition(513, playTime);
  servo2.setPosition(403, playTime);
  servo3.setPosition(479, playTime);
  servo4.setPosition(405, playTime);
  servo5.setPosition(222, playTime);
  servo6.setPosition(705, playTime);
}

void setup()
{
  servoSerial.begin(115200);

  servoSerial.setTimeout(10);
}

void loop()
{
  neutralPosition();
  delay(2000);
  servo3.setPosition(550, playTime);
  delay(2000);
}
