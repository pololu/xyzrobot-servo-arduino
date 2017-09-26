// This example tests some features of the library that are not used in other
// examples.  It is mainly intended for developers of the library.
//
// This example will change your servo's ACK_Policy parameter to 1 if it is not
// already.

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

XYZrobotServo servo(servoSerial, 1);

void runTest()
{
  // Get the status to make sure the servo is there.
  servo.readStatus();
  if (servo.getLastError())
  {
    Serial.print(F("Error reading status: "));
    Serial.println(servo.getLastError());
    return;
  }

  // Make sure we can read and write the ACK_Policy in RAM.  Use extra delays to
  // make sure the unneeded ACKs are done before the next command.
  XYZrobotServoAckPolicy policy;
  servo.writeAckPolicyRam(XYZrobotServoAckPolicy::All);
  delay(10);
  policy = servo.readAckPolicyRam();
  if (policy != XYZrobotServoAckPolicy::All)
  {
    Serial.print(F("Error: RAM ACK_Policy is wrong (1): "));
    Serial.println((uint8_t)policy);
    return;
  }
  servo.writeAckPolicyRam(XYZrobotServoAckPolicy::OnlyReadAndStat);
  delay(10);
  policy = servo.readAckPolicyRam();
  if (policy != XYZrobotServoAckPolicy::OnlyReadAndStat)
  {
    Serial.print(F("Error: RAM ACK_Policy is wrong (2): "));
    Serial.println((uint8_t)policy);
    return;
  }

  // Make sure we can read and write ACK_Policy in EEPROM.
  // Extra delays are added because the servo cannot receive commands until it
  // is done writing to EEPROM.
  servo.writeAckPolicyEeprom(XYZrobotServoAckPolicy::All);
  delay(10);
  policy = servo.readAckPolicyEeprom();
  if (policy != XYZrobotServoAckPolicy::All)
  {
    Serial.print(F("Error: EEPROM ACK_Policy is wrong (1): "));
    Serial.println((uint8_t)policy);
    return;
  }
  servo.writeAckPolicyEeprom(XYZrobotServoAckPolicy::OnlyReadAndStat);
  delay(10);
  policy = servo.readAckPolicyEeprom();
  if (policy != XYZrobotServoAckPolicy::OnlyReadAndStat)
  {
    Serial.print(F("Error: EEPROM ACK_Policy is wrong (2): "));
    Serial.println((uint8_t)policy);
    return;
  }

  // Make sure reboot works: we set ACK_Policy in RAM to All and then expect it
  // to get rolled back.
  servo.writeAckPolicyRam(XYZrobotServoAckPolicy::All);
  delay(10);
  Serial.println(F("Rebooting servo..."));
  servo.reboot();
  delay(2500);
  policy = servo.readAckPolicyRam();
  if (policy != XYZrobotServoAckPolicy::OnlyReadAndStat)
  {
    Serial.print(F("Error: RAM ACK_Policy is wrong after reboot: "));
    Serial.println((uint8_t)policy);
    return;
  }

  Serial.println(F("Test passed."));
}

void setup()
{
  servoSerial.begin(115200);
}

void loop()
{
  delay(2000);
  runTest();
}

