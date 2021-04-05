#ifndef QUAD_COPTER_CONTROL
#define QUAD_COPTER_CONTROL

#include "Sensors.h"
#include "MultiMotorControl.h"
#include "QuadCopterTypes.h"
#include "Coordinates.h"

class QuadCopterControl
{
public:
    QuadCopterControl(uint8_t *motorPins, QuadCopterTypes configuration);
    ~QuadCopterControl();
    void Initilize();
    void Roll(int8_t degrees);
    void Pitch(int8_t degrees);
    void Yaw(int8_t degrees);
private:
    Sensors *sensors;
    MultiMotorControl *motors;
    Coordinates gyroPosition;
    int8_t pitch;
    int8_t yaw;
    int8_t roll;
};
#endif