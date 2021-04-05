#include "QuadCopterControl.h"
#include "Sensors.h"
#include "MultiMotorControl.h"
#include "QuadCopterTypes.h"

QuadCopterControl::QuadCopterControl(uint8_t *motorPins, QuadCopterTypes configuration)
{
    switch (configuration)
    {
    case Plus:
    {
    }
    break;
    case X:
    {
    }
    break;
    default:
        break;
    }

    motors = new MultiMotorControl(4, motorPins, false);
    sensors = new Sensors();
    roll = 0;
    yaw = 0;
    pitch = 0;
}

void QuadCopterControl::Initilize()
{
    sensors->initializeAll();
    gyroPosition = sensors->getGyroscopeDirection();
}

void QuadCopterControl::Yaw(int8_t degrees)
{
    yaw = degrees;
}

void QuadCopterControl::Roll(int8_t degrees)
{
    roll = degrees;
}

void QuadCopterControl::Pitch(int8_t degrees)
{
    pitch = degrees;
}

QuadCopterControl::~QuadCopterControl()
{
    delete motors;
    delete sensors;
}
