#ifndef QUAD_COPTER_CONTROL
#define QUAD_COPTER_CONTROL

#include "Sensors.h"
#include "MultiMotorControl.h"
#include "QuadCopterTypes.h"
#include "Coordinates.h"
#include "ControlModel.h"
#include "PIDModel.h"
#include "ControlErrors.h"

class QuadCopterControl
{
public:
    QuadCopterControl(uint8_t *motorPins, QuadCopterTypes configuration, float mass, float gravity = 9.8);
    void Initilize();
    void Roll(int8_t degrees);
    void Pitch(int8_t degrees);
    void Yaw(int8_t degrees);
    void Thrust(int8_t power);
    void RunControlLoop();

private:
    void ApplyPitch(float controlPitch);
    void ApplyYaw(float controlYaw);
    void ApplyRoll(float controlRoll);
    void ApplyThrottle();
    ControlErrors GetControlErrors();
    float CalculateGravityCompensation(float mass, float gravity, float pitch, float roll);
    Sensors *sensors;
    MultiMotorControl *motors;
    ControlModel imuControlValues;
    ControlModel remoteControlValues;
    PIDModel *rollPid;
    PIDModel *pitchPid;
    PIDModel *yawPid;
    MathFunctions functions;
    float mass;
    float gravity = 9.8;
    float *motorThrottles;
};
#endif
