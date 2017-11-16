// This example shows how to change the baud rate of a servo.
//
// Before using this, be sure to set the parameters below.

#include <XYZrobotServo.h>

// Change this to be the ID of the servo.
const uint8_t servoId = 1;

// Change this to be the baud rate that the servo currently uses.
// The options are 9600, 19200, 57600, or 115200.
const XYZrobotServoBaudRate servoBaudOld = XYZrobotServoBaudRate::B115200;

// Change this to be the baud rate that you want the servo to use.
const XYZrobotServoBaudRate servoBaudNew = XYZrobotServoBaudRate::B115200;

// On boards with a hardware serial port available for use, use
// that port. For other boards, create a SoftwareSerial object
// using pin 10 to receive (RX) and pin 11 to transmit (TX).
#ifdef SERIAL_PORT_HARDWARE_OPEN
#define servoSerial SERIAL_PORT_HARDWARE_OPEN
const bool usingSoftwareSerial = false;
#else
#include <SoftwareSerial.h>
SoftwareSerial servoSerial(10, 11);
const bool usingSoftwareSerial = true;
#endif

XYZrobotServo servo(servoSerial, servoId);

bool success;
char errorMessage[256];

void useOldBaudRate()
{
  servoSerial.begin(XYZrobotServoBaudRateToInt(servoBaudOld));
  delay(10);
}

void useNewBaudRate()
{
  servoSerial.begin(XYZrobotServoBaudRateToInt(servoBaudNew));
  delay(10);
}

void tryToChangeBaud()
{
  success = false;
  errorMessage[0] = 0;

  // We can't reliably receive data at the old baud rate if we
  // are using software serial and the old baud rate is 115200.
  const bool canReceiveAtOldBaud = !(usingSoftwareSerial &&
    servoBaudOld == XYZrobotServoBaudRate::B115200);

  // Switch to the old baud rate.
  useOldBaudRate();

  // Make sure we can communicate with the servo using the old
  // baud rate.
  if (canReceiveAtOldBaud)
  {
    servo.readStatus();
    if (servo.getLastError())
    {
      sprintf_P(errorMessage,
        PSTR("Could not communicate with the servo: code %d."),
        servo.getLastError());
      return;
    }
  }

  // Set the ACK policy to its default so we can later read back
  // values from EEPROM.
  servo.writeAckPolicyRam(XYZrobotServoAckPolicy::OnlyReadAndStat);

  // Make sure there is not another servo using the new baud rate
  // and same ID, because then changing the baud rate would cause
  // a conflict and it could be hard to fix.
  useNewBaudRate();
  servo.readStatus();
  if (servo.getLastError() != (uint8_t)XYZrobotServoError::HeaderTimeout)
  {
    sprintf_P(errorMessage,
      PSTR("There was already a servo at the new baud rate: code %d."),
      servo.getLastError());
    return;
  }

  // Change the baud rate in EEPROM.  (It is not in RAM.)
  useOldBaudRate();
  servo.writeBaudRateEeprom(servoBaudNew);
  delay(20);

  // Make sure the baud rate in EEPROM is correct.
  if (canReceiveAtOldBaud)
  {
    XYZrobotServoBaudRate baudFromEeprom = servo.readBaudRateEeprom();
    if (servo.getLastError())
    {
      sprintf_P(errorMessage,
        PSTR("Failed to read baud rate from EEPROM: code %d"),
        servo.getLastError());
      return;
    }
    if (baudFromEeprom != servoBaudNew)
    {
      sprintf_P(errorMessage,
        PSTR("The baud rate in EEPROM is incorrect: %d"),
        (uint8_t)baudFromEeprom);
      return;
    }
  }

  // Restart the servo so it can use its new baud rate.
  servo.reboot();
  delay(2500);

  // Make sure the servo is responding at the new baud rate.
  useNewBaudRate();
  servo.readStatus();
  if (servo.getLastError())
  {
    sprintf_P(errorMessage,
      PSTR("The servo did not respond at its new baud rate: code %d"),
      servo.getLastError());
    return;
  }

  success = true;
}

void setup()
{
  Serial.begin(115200);

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

  tryToChangeBaud();
}

void loop()
{
  if (success)
  {
    Serial.println("Successfully changed the servo's baud rate.");
  }
  else
  {
    Serial.print(F("Error: "));
    Serial.println(errorMessage);
  }

  delay(2000);
}
