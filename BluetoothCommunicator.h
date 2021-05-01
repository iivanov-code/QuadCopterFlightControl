#ifndef BLUETOOTH
#define BLUETOOTH

class BluetoothCommunicator {
  public:
    void CheckForCommands(void (*callback)(BluetoothPacket*));
    BluetoothCommunicator(uint16_t dataRate = 9600);
    ~BluetoothCommunicator();
  private:
    BluetoothPacket* ReadFromBluetooth();
    uint8_t CalculateCRC(uint8_t* dataBuff, uint8_t dataSize);
    void CheckCommandResponse(BluetoothPacket* responsePacket, int responseCode);
    BluetoothPacket* CopyPacket(BluetoothPacket* packet);
    void SendPacket(BluetoothPacket* responsePacket, bool isForValidity = false);
    void FlushExessData();
};


enum CommandTypes : uint8_t
{
    ROLL,
    THROTTLE,
    PITCH,
    YAW,
    ALL,
    INVALID
};

struct BluetoothPacket
{
  BluetoothPacket() {}
  BluetoothPacket(CommandTypes command)
  {
    Command = command;
  }

  uint8_t BeginCommand = 0x1B;
  CommandTypes Command;
  uint8_t CommandDataSize = 0;
  uint8_t* CommandData;
  union {
    uint8_t CRC;
    bool IsValid = false;
  };
};

#endif


