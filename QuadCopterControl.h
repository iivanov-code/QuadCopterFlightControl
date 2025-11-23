#ifndef QUAD_COPTER_CONTROL
#define QUAD_COPTER_CONTROL

#include "BaseQuadCopterControl.cpp"

class QuadCopterControl : public BaseQuadCopterControl
{
public:
    QuadCopterControl(uint8_t *motorPins);

protected:
    ControlErrors GetControlErrors() override
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
};
#endif
