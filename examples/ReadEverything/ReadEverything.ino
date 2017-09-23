// This example reads all the information from a servo and prints it to the
// serial monitor.

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

const uint8_t servoId = 1;

PololuSmartServo servo(servoSerial, servoId);

void setup()
{
  servoSerial.begin(115200);

  servoSerial.setTimeout(10);
}

void readEverything(PololuSmartServo & servo)
{
  PololuSmartServoStatus status = servo.readStatus();
  if (servo.getLastError())
  {
    Serial.print(F("error reading status: "));
    Serial.println(servo.getLastError());
  }
  else
  {
    Serial.println(F("status:"));
    Serial.print(F("  status error: 0x"));
    Serial.println(status.statusError, HEX);
    Serial.print(F("  status detail: 0x"));
    Serial.println(status.statusDetail, HEX);
    Serial.print(F("  pwm: "));
    Serial.println(status.pwm);
    Serial.print(F("  posRef: "));
    Serial.println(status.posRef);
    Serial.print(F("  position: "));
    Serial.println(status.position);
    Serial.print(F("  iBus: "));
    Serial.println(status.iBus);
  }

  uint8_t ram[80];
  servo.ramRead(0, ram, 30);
  if (!servo.getLastError()) { servo.ramRead(30, ram + 30, 30); }
  if (!servo.getLastError()) { servo.ramRead(60, ram + 60, 20); }
  if (servo.getLastError())
  {
    Serial.print(F("error reading RAM: "));
    Serial.println(servo.getLastError());
  }
  else
  {
    Serial.println(F("RAM:"));
    Serial.print(F("  sID: "));
    Serial.println(ram[0]);
    Serial.print(F("  ACK_Policy: "));
    Serial.println(ram[1]);
    Serial.print(F("  Alarm_LED_Policy: "));
    Serial.println(ram[2]);
    Serial.print(F("  Torque_Policy: "));
    Serial.println(ram[3]);
    Serial.print(F("  SPDctrl_Policy: "));
    Serial.println(ram[4]);
    Serial.print(F("  Max_Temperature: "));
    Serial.println(ram[5]);
    Serial.print(F("  Min_Voltage: "));
    Serial.println(ram[6]);
    Serial.print(F("  Max_Voltage: "));
    Serial.println(ram[7]);
    Serial.print(F("  Acceleration_Ratio: "));
    Serial.println(ram[8]);
    Serial.print(F("  Max_Wheel_Ref_Position: "));
    Serial.println(ram[12]);
    Serial.print(F("  Max_PWM: "));
    Serial.println(ram[16] + (ram[17] << 8));
    Serial.print(F("  Overload_Threshold: "));
    Serial.println(ram[18] + (ram[19] << 8));
    Serial.print(F("  Min_Position: "));
    Serial.println(ram[20] + (ram[21] << 8));
    Serial.print(F("  Max_Position: "));
    Serial.println(ram[22] + (ram[23] << 8));
    Serial.print(F("  Position_Kp: "));
    Serial.println(ram[24] + (ram[25] << 8));
    Serial.print(F("  Position_Kd: "));
    Serial.println(ram[26] + (ram[27] << 8));
    Serial.print(F("  Position_Ki: "));
    Serial.println(ram[28] + (ram[29] << 8));
    Serial.print(F("  Close_to_Open_Ref_Position: "));
    Serial.println(ram[30] + (ram[31] << 8));
    Serial.print(F("  Open_to_Close_Ref_Position: "));
    Serial.println(ram[32] + (ram[33] << 8));
    Serial.print(F("  Ramp_Speed: "));
    Serial.println(ram[36] + (ram[37] << 8));
    Serial.print(F("  LED_Blink_Period: "));
    Serial.println(ram[38]);
    Serial.print(F("  Packet_Timeout_Detection_Period: "));
    Serial.println(ram[40]);
    Serial.print(F("  Overload_Detection_Period: "));
    Serial.println(ram[42]);
    Serial.print(F("  Inposition_Margin: "));
    Serial.println(ram[44]);
    Serial.print(F("  Over_Voltage_Detection_Period: "));
    Serial.println(ram[45]);
    Serial.print(F("  Over_Temperature_Detection_Period: "));
    Serial.println(ram[46]);
    Serial.print(F("  Calibration_Difference: "));
    Serial.println(ram[47]);
    Serial.print(F("  Status_Error: "));
    Serial.println(ram[48]);
    Serial.print(F("  Status_Detail: "));
    Serial.println(ram[49]);
    Serial.print(F("  LED_Control: "));
    Serial.println(ram[53]);
    Serial.print(F("  Voltage: "));
    Serial.println(ram[54]);
    Serial.print(F("  Temperature: "));
    Serial.println(ram[55]);
    Serial.print(F("  Current_Control_Mode: "));
    Serial.println(ram[56]);
    Serial.print(F("  Tick: "));
    Serial.println(ram[57]);
    Serial.print(F("  Joint_Position: "));
    Serial.println(ram[60] + (ram[61] << 8));
    Serial.print(F("  PWM_Output_Duty: "));
    Serial.println(ram[64] + (ram[65] << 8));
    Serial.print(F("  Bus_Current: "));
    Serial.println(ram[66] + (ram[67] << 8));
    Serial.print(F("  Position_Goal: "));
    Serial.println(ram[68] + (ram[69] << 8));
    Serial.print(F("  Position_Ref: "));
    Serial.println(ram[70] + (ram[71] << 8));
    Serial.print(F("  Omega_Goal: "));
    Serial.println(ram[72] + (ram[73] << 8));
    Serial.print(F("  Omega_Ref: "));
    Serial.println(ram[74] + (ram[75] << 8));
    Serial.print(F("  Requested_Counts: "));
    Serial.println(ram[76] + (ram[77] << 8));
    Serial.print(F("  ACK_Counts: "));
    Serial.println(ram[78] + (ram[79] << 8));
  }
  Serial.println();
}

void loop()
{
  delay(4000);
  readEverything(servo);
}
