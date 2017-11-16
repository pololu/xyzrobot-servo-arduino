// This example shows how to make the XYZrobot 6 DOF robotic arm
// hold itself up, while complying when you try to move it by
// hand.
//
// Note that this sketch changes the Max_PWM parameter in the RAM
// of your servos.  If your servos are not responding very well
// after running this sketch, you might need to power cycle them
// to make them reload the Max_PWM parameter from EEPROM.
//
// If you send an 'f' character with the Serial Monitor, this
// sketch will print the positions of all the servos, separated
// by commas.  These numbers can then be copied into another
// program to be frames in an animation.
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

void setup()
{
  Serial.begin(115200);

  servoSerial.begin(115200);
  servoSerial.setTimeout(20);

  // To receive data, a pull-up is needed on the RX line because
  // the servos do not pull the line high while idle.  If you are
  // using SoftwareSerial, the pull-up is probably enabled
  // already.  If you are using the hardware serial port on an
  // ATmega32U4-based board, we know the RX pin must be pin 0 so
  // we enable its pull-up here.  For other cases, you should add
  // code below to enable the pull-up on your board's RX line.
#if defined(SERIAL_PORT_HARDWARE_OPEN) && defined(__AVR_ATmega32U4__)
  pinMode(0, INPUT_PULLUP);
#endif
}

bool updateServo(XYZrobotServo & servo)
{
  XYZrobotServoStatus status = servo.readStatus();
  if (servo.getLastError())
  {
    return false;
  }

  // If posRef (the position that the servo is trying to
  // maintain) is more than servoHysteresis away from the current
  // measured position, then the servo is probably being
  // manipulated by hand, so move posRef.
  int16_t newPosRefSigned = constrain((int16_t)status.posRef,
    (int16_t)(status.position - servoHysteresis),
    (int16_t)(status.position + servoHysteresis));

  // Convert posRef back to an unsigned number, handling the case
  // where it is negative.
  uint16_t newPosRef = newPosRefSigned < 0 ? 0 : newPosRefSigned;

  servo.setPosition(newPosRef);

  servo.writeMaxPwmRam(maxPwm);

  return true;
}

void updateServos()
{
  for (uint8_t i = 1; i < SERVO_COUNT; i++)
  {
    bool success = updateServo(*servos[i]);
    if (!success)
    {
      Serial.print(F("Error: Failed to communicate with servo "));
      Serial.println(servos[i]->getId());
      break;
    }
  }
}

void handleSerialCommands()
{
  int input = Serial.read();

  // If we receive an 'f' from the serial monitor, print the
  // current servo positions so they can be used to make an
  // animation.
  if (input == 'f')
  {
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
    {
      uint16_t posRef = servos[i]->readPosRef();
      Serial.print(posRef);
      if (i + 1 != SERVO_COUNT)
      {
        Serial.print(F(", "));
      }
    }
    Serial.println();
  }
}


void loop()
{
  updateServos();

  handleSerialCommands();

  delay(20);
}
