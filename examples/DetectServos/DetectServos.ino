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

void setup()
{
  // TODO: need to try the other 3 possible baud rates too
  servoSerial.begin(115200);

  // Set the timeout to something short so we are not waiting a long time for
  // non-existent servos to respond.
  servoSerial.setTimeout(10);
}

void loop()
{
  delay(2000);

  Serial.println(F("Detecting servos..."));

  // The A1-16 IDs go from 1 to 20.
  for (uint16_t id = 1; id <= 20; id++)
  {
    delay(10);
    servoSerial.flush();

    XYZrobotServo servo(servoSerial, id);
    servo.readStatus();

    if (!servo.getLastError())
    {
      Serial.print(F("ID "));
      Serial.print(id);
      Serial.println(F(": detected servo"));
    }
    else if (servo.getLastError() == (uint8_t)XYZrobotServoError::HeaderTimeout)
    {
      // This is the error we get if there was no response at all.
      // Most of the IDs will have this error, so don't print anything.
    }
    else
    {
      Serial.print(F("ID "));
      Serial.print(id);
      Serial.print(F(": error "));
      Serial.println(servo.getLastError());
    }
  }
}
