#include "QuadCopterControl.h"

QuadCopterControl::QuadCopterControl(uint8_t *motorPins, float _mass, float _gravity)
{
    int motorsCount = 4;

    gravity = _gravity;
    mass = _mass;
    functions = new MathFunctions();
    motors = new MultiMotorControl(motorsCount, motorPins, false);
    motorThrottles = new float[motorsCount];
    sensors = new Sensors();
    rollPid = new PIDModel();
    pitchPid = new PIDModel();
    yawPid = new PIDModel();

    imuControlValues.Roll = 0;
    imuControlValues.Yaw = 0;
    imuControlValues.Pitch = 0;

    remoteControlValues.Roll = 0;
    remoteControlValues.Yaw = 0;
    remoteControlValues.Pitch = 0;
}

void QuadCopterControl::Initilize()
{
    sensors->initializeAll();
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
    float controlRoll = functions->CalculatePID(imuControlValues.Roll, remoteControlValues.Roll, rollPid);
    float controlPitch = functions->CalculatePID(imuControlValues.Pitch, remoteControlValues.Pitch, pitchPid);
    float controlYaw = functions->CalculatePID(imuControlValues.Yaw, remoteControlValues.Yaw, yawPid);

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
    float mgFactor = CalculateGravityCompensation(mass, gravity, imuControlValues.Pitch, imuControlValues.Roll);
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

float QuadCopterControl::CalculateGravityCompensation(float mass, float gravity, float pitch, float roll)
{
    return (mass * gravity / 4) * cos(pitch) * cos(roll);
}