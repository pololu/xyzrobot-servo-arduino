#include <PololuSmartServo.h>

// TODO: no globals
uint8_t pkt_buf[107];
PololuSmartServo::Status status;

PololuSmartServo::PololuSmartServo(Stream & stream, uint8_t id)
{
  this->stream = &stream;
  this->id = id;
}

void PololuSmartServo::sendCmd(uint8_t cmd, const uint8_t * data, uint8_t data_size)
{
  uint8_t header[7];
  uint8_t size = data_size + sizeof(header);

  header[0] = 0xFF;
  header[1] = 0xFF;
  header[2] = size;
  header[3] = id;
  header[4] = cmd;

  uint8_t checksum = size ^ id ^ cmd;
  for (uint8_t i = 0; i < data_size; i++)
  {
    checksum ^= data[i];
  }

  header[5] = checksum & 0xFE;
  header[6] = ~checksum & 0xFE;

  stream->write(header, sizeof(header));
  stream->write(data, data_size);
}

void PololuSmartServo::readAck(uint8_t exp_cmd,
  const uint8_t * data, uint8_t exp_data_size)
{
  // TODO: this check should not be needed
  if (exp_data_size > 100) { lastError = 0; return; }
  uint8_t exp_size = exp_data_size + 7;

  // TODO: should just use the timeout feature of the stream object instead
  const uint16_t timeout = 10;
  uint16_t start_ms = millis();

  while (stream->available() < 7)
  {
    if ((uint16_t)(millis() - start_ms) >= timeout)
    {
      lastError = 2;
      return;
    }
  }

  uint8_t start_bytes_seen = 0;
  while ((start_bytes_seen < 2) && (uint16_t)((millis() - start_ms) < timeout))
  {
    if (stream->read() == 0xFF) { start_bytes_seen++; }
    else { start_bytes_seen = 0; }
  }

  if (start_bytes_seen != 2)
  {
    lastError = 3;
    return;
  }

  uint8_t size = stream->read();
  if (size != exp_size)
  {
    lastError = 4;
    return;
  }

  if (stream->readBytes(pkt_buf + 3, size - 3) != (size - 3) ||
      pkt_buf[3] != id ||
      pkt_buf[4] != exp_cmd)
  {
    lastError = 1;
    return;
  }

  uint8_t checksum1;
  uint8_t checksum2;

  checksum1 = exp_size ^ id ^ exp_cmd;

  for (uint8_t i = 0; i < exp_data_size; i++)
  {
    //data[i] = pkt_buf[7 + i];
    checksum1 ^= data[i];
  }

  checksum1 &= 0xFE;
  checksum2 = (~checksum1) & 0xFE;

  if (pkt_buf[5] != checksum1 ||
      pkt_buf[6] != checksum2)
  {
    lastError = 4;
    return;
  }
}

void PololuSmartServo::readStatus()
{
  lastError = 0;
  sendCmd(0x07, NULL, 0);
  readAck(0x47, (uint8_t *)&status, 10);
}
