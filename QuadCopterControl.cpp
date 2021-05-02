#include "QuadCopterControl.h"

QuadCopterControl::QuadCopterControl(uint8_t *motorPins)
{
    motors = new MultiMotorControl(ConstConfig::MOTORS_COUNT, motorPins, false);
    motorThrottles = new float[ConstConfig::MOTORS_COUNT];
    sensors = new Sensors();
    rollPid = new PIDModel();
    pitchPid = new PIDModel();
    yawPid = new PIDModel();
}

void QuadCopterControl::Initilize()
{
    sensors->InitializeAll();
}

void QuadCopterControl::Yaw(int8_t targetDegrees)
{
    remoteControlValues.Yaw = targetDegrees;
}

void QuadCopterControl::Roll(int8_t targetDegrees)
{
    remoteControlValues.Roll = targetDegrees;
}

void QuadCopterControl::Pitch(int8_t targetDegrees)
{
    remoteControlValues.Pitch = targetDegrees;
}

void QuadCopterControl::Thrust(int8_t power)
{
    remoteControlValues.Throttle = power;
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

void QuadCopterControl::ApplyPitch(float controlPitch)
{
    short throttle = (short)controlPitch;
    motorThrottles[0] += throttle;
    motorThrottles[1] += throttle;
    motorThrottles[2] -= throttle;
    motorThrottles[3] -= throttle;
}

void QuadCopterControl::ApplyRoll(float controlRoll)
{
    short throttle = (short)controlRoll;
    motorThrottles[0] += throttle;
    motorThrottles[1] -= throttle;
    motorThrottles[2] -= throttle;
    motorThrottles[3] += throttle;
}

void QuadCopterControl::ApplyYaw(float controlYaw)
{
    short throttle = (short)controlYaw;
    motorThrottles[0] += throttle;
    motorThrottles[1] -= throttle;
    motorThrottles[2] += throttle;
    motorThrottles[3] -= throttle;
}

void QuadCopterControl::ApplyThrottle()
{
    float mgFactor = CalculateGravityCompensation(imuControlValues.Pitch, imuControlValues.Roll);
    float throttle = remoteControlValues.Throttle + mgFactor;

    motorThrottles[0] = throttle;
    motorThrottles[1] = throttle;
    motorThrottles[2] = throttle;
    motorThrottles[3] = throttle;
}

void QuadCopterControl::RunControlLoop()
{
    ApplyThrottle();
    ControlErrors errors = GetControlErrors();
    ApplyPitch(errors.ControlPitch);
    ApplyRoll(errors.ControlRoll);
    ApplyYaw(errors.ControlYaw);

    motors->getMotor(0)->ChangeThrottle((short)motorThrottles[0]);
    motors->getMotor(1)->ChangeThrottle((short)motorThrottles[1]);
    motors->getMotor(2)->ChangeThrottle((short)motorThrottles[2]);
    motors->getMotor(3)->ChangeThrottle((short)motorThrottles[3]);
}

float QuadCopterControl::CalculateGravityCompensation(float pitch, float roll)
{
    return (ConstConfig::QUAD_MASS * ConstConfig::GRAVITY / 4) * cos(pitch) * cos(roll);
}