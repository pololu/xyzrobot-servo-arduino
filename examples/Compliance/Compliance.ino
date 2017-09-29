// This example shows how to make the XYZrobot 6 DOF Robot Arm
// Kit hold it self up, while complying when you try to move it
// by hand.
//
// You can use this sketch with other arrangements of A1-16
// servos that are not the robot arm, but you might need to
// change the part of the code that defines SERVO_COUNT and sets
// up the XYZrobotServo objects.

#include <XYZrobotServo.h>

// This is the maximum PWM value, out of 1023, to use when
// driving the servos.  Setting it lower makes it easier to
// manipulate the arm by hand, but if you set it too low, the arm
// will not be able to hold itself up.
const uint16_t maxPwm = 80;

// This is how much the arm's position measurement has to differ
// from its target position before this sketch adjusts its target
// position.  If you set it too low, then the deflections caused
// by gravity will be large enough to make the arm move, and it
// will not hold itself up.  If you set it to high, it will be
// hard to accurately position the arm.
const uint16_t servoHysteresis = 5;

// On boards with a hardware serial port available for use, use
// that port. For other boards, create a SoftwareSerial object
// using pin 10 to receive (RX) and pin 11 to transmit (TX).
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define servoSerial SERIAL_PORT_HARDWARE_OPEN
#else
#include <SoftwareSerial.h>
SoftwareSerial servoSerial(10, 11);
#endif

#define SERVO_COUNT 6

XYZrobotServo servo1(servoSerial, 1);
XYZrobotServo servo2(servoSerial, 2);
XYZrobotServo servo3(servoSerial, 3);
XYZrobotServo servo4(servoSerial, 4);
XYZrobotServo servo5(servoSerial, 5);
XYZrobotServo servo6(servoSerial, 6);

XYZrobotServo * servos[SERVO_COUNT] = {
  &servo1, &servo2, &servo3, &servo4, &servo5, &servo6
};

struct ServoInfo
{
};

ServoInfo servoInfos[SERVO_COUNT];

void setup()
{
  servoSerial.begin(115200);
  servoSerial.setTimeout(10);
}

int16_t doHysteresis(int16_t current, int16_t input, int16_t hysteresis)
{
  if (current < input - hysteresis)
  {
    current = input - hysteresis;
  }

  if (current > input + hysteresis)
  {
    current = input + hysteresis;
  }

  return current;
}

bool updateServo(XYZrobotServo & servo, ServoInfo & info)
{
  XYZrobotServoStatus status = servo.readStatus();
  if (servo.getLastError())
  {
    return false;
  }

  int16_t newPosRefSigned = doHysteresis(
    status.posRef, status.position, servoHysteresis);
  uint16_t newPosRef = newPosRefSigned < 0 ? 0 : newPosRefSigned;
  servo.setPosition(newPosRef);

  servo.writeMaxPwmRam(maxPwm);

  return true;
}

void loop()
{
  for (uint8_t i = 0; i < SERVO_COUNT; i++)
  {
    bool success = updateServo(*servos[i], servoInfos[i]);
    if (!success)
    {
      Serial.print(F("Error: Failed to communicate with servo "));
      Serial.println(servos[i]->getId());
      break;
    }
  }

  delay(20);
}
