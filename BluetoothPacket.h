#ifndef BLUETOOTH_PACKET
#define BLUETOOTH_PACKET

#include "Arduino.h"

enum CommandTypes
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
    uint8_t *CommandData;
    union
    {
        uint8_t CRC;
        bool IsValid = false;
    };
};

#endif