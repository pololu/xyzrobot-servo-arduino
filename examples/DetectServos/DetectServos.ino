// This sketch searches for smart servos and prints information
// about them that will be useful for communicating with them.
//
// Also, when it detects a servo, it makes the servo turn its LED
// magenta for one second.  Since servos are detected in order by
// increasing ID number, the blinking LED can help you verify
// that your servos have the correct IDs.

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

// By default, only search for servos with IDs from 1 to 20, to
// save time.  If you think you might have a servo with a higher
// ID, you can change this to 255.
const uint8_t maxId = 20;

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
  digitalWrite(0, HIGH);
#endif
}

void blinkServoLed(XYZrobotServo & servo)
{
  // Make all the LEDs be user-controlled.
  servo.writeAlarmLedPolicyRam(0b1111);

  // Turn on the red and blue LEDs to make magenta.
  servo.writeLedControl(0b1010);

  delay(1000);

  // Turn on the white LED while turning the others off, and
  // restore control to the system.  Both the writeLedControl and
  // the delay commands seem to be necessary to get the LED to
  // change back to white.
  servo.writeLedControl(0b0001);
  delay(10);
  servo.writeAlarmLedPolicyRam(0);
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
      // This is the error we get if there was no response at
      // all.  Most of the IDs will have this error, so don't
      // print anything.
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

  // We successfully detected the servo.
  Serial.print(F("ID "));
  Serial.print(id);
  Serial.println(F(": detected servo"));

  // Make the servo's LED shine magenta for one second.  You can
  // comment this out if you want to speed up this sketch.
  blinkServoLed(servo);

  // Print some other information that will be useful when
  // communicating with it or troubleshooting issues.

  XYZrobotServoAckPolicy ackPolicy = servo.readAckPolicyRam();
  Serial.print(F("  ACK policy: "));
  Serial.println((uint8_t)ackPolicy);

  XYZrobotServoAckPolicy ackPolicyEeprom = servo.readAckPolicyEeprom();
  if (ackPolicyEeprom != ackPolicy)
  {
    Serial.print(F("  ACK policy (EEPROM): "));
    Serial.println((uint8_t)ackPolicyEeprom);
  }

  uint8_t versionInfo[4] = { 0, 0, 0, 0 };
  servo.eepromRead(0, versionInfo, sizeof(versionInfo));
  Serial.print(F("  Version info: "));
  Serial.print(versionInfo[0]);  // Model_No
  Serial.print(',');
  Serial.print(versionInfo[1]);  // Year
  Serial.print('-');
  Serial.print(versionInfo[2] & 0xF);  // Month
  Serial.print('-');
  Serial.print(versionInfo[3]);  // Day
  Serial.print(',');
  Serial.println(versionInfo[2] >> 4 & 0xF);  // Firmware version

}

void detectServos(uint32_t baudRate)
{
  servoSerial.begin(baudRate);

  Serial.print(F("Detecting servos at "));
  Serial.print(baudRate);
  Serial.println(F(" baud..."));
  delay(10);

  for (uint16_t id = 1; id <= maxId; id++)
  {
    detectServo(id);
  }
}

void loop()
{
  delay(2500);

  // Try each of the four baud rates supported by the A1-16 servo.
  detectServos(9600);
  detectServos(19200);
  detectServos(57600);
  detectServos(115200);
}
