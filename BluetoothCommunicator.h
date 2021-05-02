#ifndef BLUETOOTH_COMMUNICATOR
#define BLUETOOTH_COMMUNICATOR

#include "BluetoothPacket.h"
#include "Arduino.h"

class BluetoothCommunicator
{
public:
  void CheckForCommands(void (*callback)(BluetoothPacket *packet));
  BluetoothCommunicator(uint16_t dataRate = 9600);
  ~BluetoothCommunicator();

private:
  BluetoothPacket *ReadFromBluetooth();
  uint8_t CalculateCRC(uint8_t *dataBuff, uint8_t dataSize);
  void CheckCommandResponse(BluetoothPacket *responsePacket, int responseCode);
  BluetoothPacket *CopyPacket(BluetoothPacket *packet);
  void SendPacket(BluetoothPacket *responsePacket, bool isForValidity = false);
  void FlushExessData();
};

#endif
