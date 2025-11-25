#include "QuadCopterControl.h"

QuadCopterControl::QuadCopterControl(uint8_t *motorPins) : BaseQuadCopterControl(motorPins)
{
    // Base class constructor handles initialization
}

QuadCopterControl::~QuadCopterControl()
{
}

ControlErrors QuadCopterControl::GetControlErrors()
{
    float controlRoll = MathFunctions::CalculatePID(imuControlValues.Roll, remoteControlValues.Roll, rollPid);
    float controlPitch = MathFunctions::CalculatePID(imuControlValues.Pitch, remoteControlValues.Pitch, pitchPid);
    float controlYaw = MathFunctions::CalculatePID(imuControlValues.Yaw, remoteControlValues.Yaw, yawPid);

    ControlErrors errors;
    errors.ControlRoll = controlRoll;
    errors.ControlPitch = controlPitch;
    errors.ControlYaw = controlYaw;

    return errors;
}