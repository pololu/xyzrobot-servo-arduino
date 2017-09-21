#include <PololuSmartServo.h>

#define CMD_REQ_EEPROM_WRITE 0x01
#define CMD_REQ_EEPROM_READ  0x02
#define CMD_REQ_RAM_WRITE    0x03
#define CMD_REQ_RAM_READ     0x04
#define CMD_REQ_I_JOG        0x05
#define CMD_REQ_S_JOG        0x06
#define CMD_REQ_STAT         0x07
#define CMD_REQ_ROLLBACK     0x08
#define CMD_REQ_REBOOT       0x09

PololuSmartServo::PololuSmartServo(Stream & stream, uint8_t id)
{
  this->stream = &stream;
  this->id = id;
  this->lastError = 0;
}

void PololuSmartServo::sendRequest(uint8_t cmd, const uint8_t * data, uint8_t dataSize)
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
}

void PololuSmartServo::readAck(uint8_t cmd, uint8_t * data, uint8_t dataSize)
{
  // The CMD byte for an acknowledgment always has bit 6 set.
  cmd |= 0x40;

  uint8_t header[7];

  uint8_t size = dataSize + sizeof(header);

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

  byteCount = stream->readBytes(data, dataSize);
  if (byteCount != dataSize)
  {
    lastError = 7;
    return;
  }

  uint8_t checksum = size ^ id ^ cmd;
  for (uint8_t i = 0; i < dataSize; i++) { checksum ^= data[i]; }

  if (header[5] != (checksum & 0xFE))
  {
    lastError = 8;
    return;
  }

  if (header[6] != (~checksum & 0xFE))
  {
    lastError = 9;
    return;
  }

  lastError = 0;
}

// TODO: no globals
PololuSmartServo::Status status;

// TODO: need to properly return the status to caller; see if just returning the
// struct is just as efficient as taking a pointer.
void PololuSmartServo::readStatus()
{
  sendRequest(CMD_REQ_STAT, NULL, 0);
  readAck(CMD_REQ_STAT, (uint8_t *)&status, 10);
}
