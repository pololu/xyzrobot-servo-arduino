#include <XYZrobotServo.h>

#define CMD_REQ_EEPROM_WRITE 0x01
#define CMD_REQ_EEPROM_READ  0x02
#define CMD_REQ_RAM_WRITE    0x03
#define CMD_REQ_RAM_READ     0x04
#define CMD_REQ_I_JOG        0x05
#define CMD_REQ_S_JOG        0x06
#define CMD_REQ_STAT         0x07
#define CMD_REQ_ROLLBACK     0x08
#define CMD_REQ_REBOOT       0x09

#define SET_POSITION_CONTROL 0
#define SET_SPEED_CONTROL 1
#define SET_TORQUE_OFF 2
#define SET_POSITION_CONTROL_SERVO_ON 3

XYZrobotServo::XYZrobotServo(Stream & stream, uint8_t id)
{
  this->stream = &stream;
  this->id = id;
  this->lastError = 0;
  this->lastStatusError = 0;
  this->lastStatusDetail = 0;
}

void XYZrobotServo::eepromRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize)
{
  memoryRead(CMD_REQ_EEPROM_READ, startAddress, data, dataSize);
}

void XYZrobotServo::ramRead(uint8_t startAddress, uint8_t * data, uint8_t dataSize)
{
  memoryRead(CMD_REQ_RAM_READ, startAddress, data, dataSize);
}

// TODO: need to properly return the status to caller; see if just returning the
// struct is just as efficient as taking a pointer.
XYZrobotServoStatus XYZrobotServo::readStatus()
{
  flushRead();

  XYZrobotServoStatus status;
  sendRequest(CMD_REQ_STAT, NULL, 0);
  readAck(CMD_REQ_STAT, (uint8_t *)&status, 10, NULL, 0);
  return status;
}

void XYZrobotServo::setTargetPosition(uint16_t position, uint8_t playTime)
{
  sendIJog(position, SET_POSITION_CONTROL, playTime);
}

void XYZrobotServo::flushRead()
{
  while(stream->available()) { stream->read(); }
}

void XYZrobotServo::sendRequest(uint8_t cmd, const uint8_t * data, uint8_t dataSize)
{
  uint8_t header[7];

  uint8_t size = dataSize + sizeof(header);

  uint8_t checksum = size ^ id ^ cmd;
  for (uint8_t i = 0; i < dataSize; i++) { checksum ^= data[i]; }

  header[0] = 0xFF;
  header[1] = 0xFF;
  header[2] = size;
  header[3] = id;
  header[4] = cmd;
  header[5] = checksum & 0xFE;
  header[6] = ~checksum & 0xFE;

  stream->write(header, sizeof(header));
  stream->write(data, dataSize);

  lastError = 0;
}

void XYZrobotServo::readAck(uint8_t cmd,
  uint8_t * data1, uint8_t data1Size,
  uint8_t * data2, uint8_t data2Size)
{
  // The CMD byte for an acknowledgment always has bit 6 set.
  cmd |= 0x40;

  uint8_t header[7];

  uint8_t size = sizeof(header) + data1Size + data2Size;

  uint8_t byteCount = stream->readBytes(header, sizeof(header));
  if (byteCount != sizeof(header))
  {
    lastError = 1;
    return;
  }

  if (header[0] != 0xFF)
  {
    lastError = 2;
    return;
  }

  if (header[1] != 0xFF)
  {
    lastError = 3;
    return;
  }

  if (header[3] != id)
  {
    lastError = 4;
    return;
  }

  if (header[4] != cmd)
  {
    lastError = 5;
    return;
  }

  if (header[2] != size)
  {
    lastError = 6;
    return;
  }

  if (data1Size)
  {
    byteCount = stream->readBytes(data1, data1Size);
    if (byteCount != data1Size)
    {
      lastError = 7;
      return;
    }
  }

  if (data2Size)
  {
    byteCount = stream->readBytes(data2, data2Size);
    if (byteCount != data2Size)
    {
      lastError = 8;
      return;
    }
  }

  uint8_t checksum = size ^ id ^ cmd;
  for (uint8_t i = 0; i < data1Size; i++) { checksum ^= data1[i]; }
  for (uint8_t i = 0; i < data2Size; i++) { checksum ^= data2[i]; }

  if (header[5] != (checksum & 0xFE))
  {
    Serial.print("hey ");
    Serial.print(header[5], HEX);
    Serial.print(' ');
    Serial.print(header[6], HEX);
    Serial.print(' ');
    Serial.println(checksum, HEX);

    lastError = 9;
    return;
  }

  if (header[6] != (~checksum & 0xFE))
  {
    lastError = 10;
    return;
  }

  lastError = 0;
}

void XYZrobotServo::memoryRead(uint8_t cmd, uint8_t startAddress,
  uint8_t * data, uint8_t dataSize)
{
  flushRead();

  uint8_t request[2];
  request[0] = startAddress;
  request[1] = dataSize;
  sendRequest(cmd, request, sizeof(request));

  uint8_t response[4];
  readAck(cmd, response, 4, data, dataSize);
  if (lastError) { return; }

  // Despite what the A1-16 datasheet says, the first two bytes of the response
  // tend to 0, and the start address and data size come after that.

  if (response[2] != request[0])
  {
    lastError = 15;
    return;
  }

  if (response[3] != request[1])
  {
    lastError = 16;
    return;
  }
}

void XYZrobotServo::sendIJog(uint16_t goal, uint8_t type, uint8_t playTime)
{
  if (goal > 1023) { goal = 1023; }
  uint8_t data[5];
  data[0] = goal & 0xFF;
  data[1] = goal >> 8 & 0xFF;
  data[2] = type;
  data[3] = id;
  data[4] = playTime;
  sendRequest(CMD_REQ_I_JOG, data, sizeof(data));

  // Assumption: The ACK_Policy setting is not 2, so this command does not
  // result in an acknowledgment.
}
