#include "../Interfaces/MultiMotorControl.h"

MultiMotorControl::MultiMotorControl(uint8_t motorsCount, uint8_t *pins, bool bidirectional)
{
    count = motorsCount;
    motors = new ThrottleControl *[motorsCount];
    for (int i = 0; i < motorsCount; i++)
    {
        motors[i] = new ThrottleControl(pins[i], bidirectional);
    }
}

MultiMotorControl::~MultiMotorControl()
{
    for (int i = 0; i < count; i++)
    {
        delete motors[i];
    }
    delete[] motors;
}

ThrottleControl *MultiMotorControl::operator[](int index)
{
    return motors[index];
}

ThrottleControl *MultiMotorControl::getMotor(uint8_t i)
{
    return motors[i];
}