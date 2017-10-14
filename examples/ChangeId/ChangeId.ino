// This example shows how to change the ID number of a servo.
//
// Before using this, be sure to set the parameters below.

// Change this to be ID that the servo currently has.
const uint8_t servoIdOld = 1;

// Change this to be the ID that you want the servo to have.
const uint8_t servoIdNew = 1;

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

XYZrobotServo servoOld(servoSerial, servoIdOld);
XYZrobotServo servoNew(servoSerial, servoIdNew);

bool success;
char errorMessage[256];

void tryToChangeId()
{
  success = false;
  errorMessage[0] = 0;

  // Make sure we can communicate with the servo with its current ID.
  servoOld.readStatus();
  if (servoOld.getLastError())
  {
    sprintf_P(errorMessage,
      PSTR("Could not communicate with the servo: code %d."),
      servoOld.getLastError());
    return;
  }

  // Set the ACK policy to its default so we can later read back
  // values from EEPROM.
  servoOld.writeAckPolicyRam(XYZrobotServoAckPolicy::OnlyReadAndStat);

  // Make sure there is not another servo using the new ID,
  // because then changing the ID would cause an ID conflict and
  // it could be hard to fix.
  servoNew.readStatus();
  if (servoNew.getLastError() != (uint8_t)XYZrobotServoError::HeaderTimeout)
  {
    sprintf_P(errorMessage,
      PSTR("There was already a servo at the new ID: code %d."),
      servoNew.getLastError());
    return;
  }

  // Change the ID in EEPROM and RAM.
  servoOld.writeIdEeprom(servoIdNew);
  delay(10);
  servoOld.writeIdRam(servoIdNew);
  delay(10);

  // Make sure the servo is responding to the new ID.
  servoNew.readStatus();
  if (servoNew.getLastError())
  {
    sprintf_P(errorMessage,
      PSTR("The servo did not respond at its new ID: code %d"),
      servoNew.getLastError());
    return;
  }

  // Make sure the ID in EEPROM is correct.
  uint8_t idFromEeprom = servoNew.readIdEeprom();
  if (servoNew.getLastError())
  {
    sprintf_P(errorMessage,
      PSTR("Failed to read ID from EEPROM: code %d"),
      servoNew.getLastError());
    return;
  }
  if (idFromEeprom != servoIdNew)
  {
    sprintf_P(errorMessage,
      PSTR("The ID in EEPROM is incorrect: %d"),
      idFromEeprom);
    return;
  }

  success = true;
}

void setup()
{
  // Turn on the serial port and set its baud rate.
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

  delay(2500);

  tryToChangeId();
}

void loop()
{
  if (success)
  {
    Serial.println("Successfully changed the servo's ID.");
  }
  else
  {
    Serial.print(F("Error: "));
    Serial.println(errorMessage);
  }

  delay(2000);
}
