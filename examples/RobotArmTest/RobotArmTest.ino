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

XYZrobotServo * servos[] = {
  &servo1, &servo2, &servo3, &servo4, &servo5, &servo6
};

void setup()
{
  // TODO: need to try the other 3 possible baud rates too
  servoSerial.begin(115200);

  servoSerial.setTimeout(10);

  servo1.setTargetPosition(513, 40);
  servo2.setTargetPosition(403, 40);
  servo3.setTargetPosition(479, 40);
  servo4.setTargetPosition(405, 40);
  servo5.setTargetPosition(222, 40);
  servo6.setTargetPosition(705, 40);
}

void loop()
{
  servo3.readStatus();

  Serial.println(F("Moving down"));
  servo3.setTargetPosition(479, 60);
  if (servo3.getLastError())
  {
    Serial.print(F("error "));
    Serial.println(servo3.getLastError());
  }
  else
  {
    Serial.println("no error");
  }

  while (0) // tmphax: read positions
  {
    delay(25);
    for (uint8_t i = 0; i < 6; i++)
    {
      XYZrobotServo::Status status = servos[i]->readStatus();
      Serial.print(status.currentPosition);
      Serial.print(' ');
    }
    Serial.println();
  }

  delay(2000);
  Serial.println(F("Moving up"));
  servo3.setTargetPosition(550, 0);
  uint16_t start = millis();
  if (servo3.getLastError())
  {
    Serial.print(F("error "));
    Serial.println(servo3.getLastError());
  }
  else
  {
    Serial.println("no error");
  }

  while (1)
  {
    XYZrobotServo::Status status = servo3.readStatus();
    Serial.print((millis() - start) & 0xFFFF);
    Serial.print(' ');
    Serial.print(status.targetPosition);
    Serial.print(' ');
    Serial.println(status.currentPosition);
    if (status.targetPosition == 550) { break; }
  }

  delay(2000);
}
