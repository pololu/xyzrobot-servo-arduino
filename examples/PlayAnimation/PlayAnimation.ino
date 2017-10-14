// This example shows how to play an animation consisting of
// pre-recorded sets of servo positions on the XYZrobot 6 DOF
// Robot Arm Kit.
//
// The animation is empty by default; you need to add frames to
// it to make it work.  You can use the Compliance example to get
// the position data for the frames of the animation and then
// copy that data into this sketch.
//
// You can use this sketch with other arrangements of A1-16
// servos that are not the robot arm, but you might need to
// change the part of the code that defines SERVO_COUNT and sets
// up the XYZrobotServo objects.

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

typedef uint16_t Frame[1 + SERVO_COUNT];

#define END_FRAME { 0xFFFF }

const Frame animation[] = {
  // To make your servos move, you need to add animation frames here.
  // Each frame is of the form:
  //
  //   { DURATION, POSITION1, POSITION2, POSITION3, ... },
  //
  // For example, if you had 3 servos, the following frame would
  // last for 1300 ms and put the first servo in position 333
  // while putting the second servo in position 555:
  //
  //   { 1300, 333, 555 },
  //
  // You can use the Compliance example to get position data that
  // can be pasted into your frames, but don't forget to add the
  // duration (in milliseconds) to the beginning.

  END_FRAME  // Our loop uses this to detect the end.
};

void setup()
{
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

void loop()
{
  delay(2500);

  // The playtime is a time in units of 10 ms that specifies how
  // long the servos should take to move to the desired position.
  // You can change it to 0 if you want the movements to be as
  // fast as possible.
  uint16_t playtime = 25;

  // Loop over each from of the animation.
  for (const Frame * frame = animation; ; frame++)
  {
    uint16_t duration = (*frame)[0];

    // Break out of the loop if this is the end frame.
    if (duration == 0xFFFF) { break; }

    // Go to the next frame if the duration of this frame is 0.
    if (duration == 0) { continue; }

    // Set the positions of each servo using the data in the frame.
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
    {
      servos[i]->setPosition((*frame)[1 + i], playtime);
    }

    delay(duration);
  }
}
