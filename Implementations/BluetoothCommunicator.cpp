#include "../Interfaces/BluetoothCommunicator.h"

BluetoothCommunicator::BluetoothCommunicator(uint16_t dataRate)
{
  Serial.begin(dataRate);
}

BluetoothCommunicator::~BluetoothCommunicator()
{
  Serial.end();
}

void BluetoothCommunicator::CheckForCommands(void (*callback)(BluetoothPacket *packet))
{
  BluetoothPacket *packet = ReadFromBluetooth();

  if (packet)
  {
    BluetoothPacket *responsePacket = CopyPacket(packet);

    if (packet->IsValid)
    {
      (*callback)(packet);
    }
    else
    {
      FlushExessData();
      responsePacket->CommandDataSize = 0;
      responsePacket->IsValid = false;
      SendPacket(responsePacket, true);
    }
  }
}

void BluetoothCommunicator::FlushExessData()
{
  while (Serial.available())
  {
    Serial.read();
  }
}

void BluetoothCommunicator::SendPacket(BluetoothPacket *responsePacket, bool isForValidity)
{
  byte dataSize = 4 + responsePacket->CommandDataSize;
  byte *dataBuffer = new byte[dataSize];

  dataBuffer[0] = responsePacket->BeginCommand;
  dataBuffer[1] = (byte)responsePacket->Command;
  dataBuffer[2] = responsePacket->CommandDataSize;

  for (int i = 3; i < responsePacket->CommandDataSize; i++)
  {
    dataBuffer[i] = responsePacket->CommandData[(i - 3)];
  }

  if (isForValidity)
  {
    dataBuffer[dataSize - 1] = responsePacket->IsValid;
  }
  else
  {
    dataBuffer[dataSize - 1] = responsePacket->CRC;
  }

  Serial.write(dataBuffer, dataSize);
  Serial.flush();
}

BluetoothPacket *BluetoothCommunicator::ReadFromBluetooth()
{
  uint8_t availableBytes = 0;

  if (Serial && (availableBytes = Serial.available()) > 0)
  {
    BluetoothPacket *packet = new BluetoothPacket();
    byte beginStream = Serial.read();
    if (packet->BeginCommand == beginStream)
    {
      packet->Command = (CommandTypes)Serial.read();
      packet->CommandDataSize = Serial.read();
      byte *dataBuffer = new byte[packet->CommandDataSize];
      packet->CommandData = dataBuffer;
      Serial.readBytes(dataBuffer, packet->CommandDataSize);
      packet->CRC = Serial.read();

      uint8_t calculatedCRC = CalculateCRC(dataBuffer, packet->CommandDataSize);

      if (calculatedCRC == packet->CRC)
      {
        packet->IsValid = true;
      }
      else
      {
        packet->IsValid = false;
      }

      return packet;
    }
    else
    {
      return new BluetoothPacket(CommandTypes::INVALID);
    }
  }
  else
  {
    return NULL;
  }
}

BluetoothPacket *BluetoothCommunicator::CopyPacket(BluetoothPacket *packet)
{
  BluetoothPacket *newPacket = new BluetoothPacket();
  newPacket->BeginCommand = packet->BeginCommand;
  newPacket->Command = packet->Command;
  return newPacket;
}

uint8_t BluetoothCommunicator::CalculateCRC(uint8_t *dataBuff, uint8_t dataSize)
{

  if (dataSize == 0)
  {
    return 0;
  }
  else
  {

    uint8_t crc = dataBuff[0];
    for (byte i = 1; i < dataSize; i++)
    {
      crc = crc ^ dataBuff[i];
    }
    return crc;
  }
}
