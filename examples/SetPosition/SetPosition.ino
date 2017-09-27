// This sketch shows how to move a servo back and forth between
// two different position.
//
// Positions are represented as numbers between 0 and 1023.  When
// you set a position, you can also specify the playtime, which
// is how long you want the movement to take, in units of 10 ms.

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
XYZrobotServo servo(servoSerial, 1);

const uint8_t playtime = 75;

void setup()
{
  servoSerial.begin(115200);

  servoSerial.setTimeout(10);
}

void loop()
{
  delay(2500);
  servo.setPosition(475, playtime);
  delay(2500);
  servo.setPosition(525, playtime);
}
