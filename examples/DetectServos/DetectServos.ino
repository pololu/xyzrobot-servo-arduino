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
  // Set the timeout to something short so we are not waiting a long time for
  // non-existent servos to respond.
  servoSerial.setTimeout(10);
}

void detectServo(uint8_t id)
{
  XYZrobotServo servo(servoSerial, id);

  // Try to read the status from the servo to see if it is there.
  servo.readStatus();
  if (servo.getLastError())
  {
    if (servo.getLastError() == (uint8_t)XYZrobotServoError::HeaderTimeout)
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
    return;
  }

  Serial.print(F("ID "));
  Serial.print(id);
  Serial.println(F(": detected servo"));

  // Also read the ACK policy, since that is important to know when
  // communicating with the servo.
  XYZrobotServoAckPolicy ackPolicy = servo.readAckPolicyRam();
  if (servo.getLastError())
  {
    if (servo.getLastError() == (uint8_t)XYZrobotServoError::HeaderTimeout)
    {
      Serial.println(F("  ACK policy: probably 0: no response received\n"));
    }
    else
    {
    Serial.print(F("  Error getting ACK policy: "));
    Serial.println(servo.getLastError());
    }
  }
  else
  {
    Serial.print(F("  ACK policy: "));
    Serial.println((uint8_t)ackPolicy);
  }
}

void detectServos(uint32_t baudRate)
{
  servoSerial.begin(baudRate);

  Serial.print(F("Detecting servos 1 to 20 at "));
  Serial.print(baudRate);
  Serial.println(F(" baud..."));
  delay(10);

  // Search through possible servo IDs.
  for (uint16_t id = 1; id <= 20; id++)
  {
    detectServo(id);
  }
}

void loop()
{
  delay(2000);

  // Try each of the four baud rates supported by the A1-16 servo.
  detectServos(9600);
  detectServos(19200);
  detectServos(57600);
  detectServos(115200);
}
