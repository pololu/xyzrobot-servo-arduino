#include <PololuSmartServo.h>

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
}

void loop()
{
  Serial.println(F("Detecting servos..."));

  // The A1-16 IDs go from 1 to 20.
  for (uint16_t id = 1; id <= 20; id++)
  {
    delay(20);
    servoSerial.flush();
    PololuSmartServo servo(servoSerial, id);
    servo.readStatus();
    uint8_t error = servo.getLastError();
    if (error == 0)
    {
      Serial.print(F("ID "));
      Serial.print(id);
      Serial.println(F(": detected servo"));
    }
  }
  delay(1000);
}
