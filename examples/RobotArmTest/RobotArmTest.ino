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

PololuSmartServo servo1(servoSerial, 1);
PololuSmartServo servo2(servoSerial, 2);
PololuSmartServo servo3(servoSerial, 3);
PololuSmartServo servo4(servoSerial, 4);
PololuSmartServo servo5(servoSerial, 5);
PololuSmartServo servo6(servoSerial, 6);

PololuSmartServo * servos[] = {
  &servo1, &servo2, &servo3, &servo4, &servo5, &servo6
};

void setup()
{
  // TODO: need to try the other 3 possible baud rates too
  servoSerial.begin(115200);
}

void loop()
{
  servo1.setTargetPosition(513, 200);
  servo2.setTargetPosition(403, 200);
  servo3.setTargetPosition(479, 200);
  servo4.setTargetPosition(405, 200);
  servo5.setTargetPosition(222, 200);
  servo6.setTargetPosition(705, 200);
  while (1) // tmphax: read the positions
  {
    delay(25);
    for (uint8_t i = 0; i < 6; i++)
    {
      PololuSmartServo::Status status;
      status = servos[i]->readStatus();
      Serial.print(status.statusError);
      Serial.print(' ');
      Serial.print(status.statusDetail);
      Serial.print(' ');
    }
    Serial.println();
  }
  delay(1000);
  servo3.setTargetPosition(550, 200);
  delay(1000);
}
