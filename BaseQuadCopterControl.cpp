#ifndef BASE_QUAD_COPTER_CONTROL
#define BASE_QUAD_COPTER_CONTROL

#include "Sensors.h"
#include "MultiMotorControl.h"
#include "QuadCopterTypes.h"
#include "Coordinates.h"
#include "ControlModel.h"
#include "PIDModel.h"
#include "ControlErrors.h"
#include "MathFunctions.h"
#include "ConstConfig.h"

class BaseQuadCopterControl
{
public:
    BaseQuadCopterControl(uint8_t *motorPins)
    {
        motors = new MultiMotorControl(ConstConfig::MOTORS_COUNT, motorPins, false);
        motorThrottles = new float[ConstConfig::MOTORS_COUNT];
        sensors = new Sensors();
        rollPid = new PIDModel();
        pitchPid = new PIDModel();
        yawPid = new PIDModel();
    }

    void Initilize()
    {
        sensors->InitializeAll();
    }

    void Yaw(int8_t targetDegrees)
    {
        remoteControlValues.Yaw = targetDegrees;
    }

    void Roll(int8_t targetDegrees)
    {
        remoteControlValues.Roll = targetDegrees;
    }

    void Pitch(int8_t targetDegrees)
    {
        remoteControlValues.Pitch = targetDegrees;
    }

    void Thrust(int8_t power)
    {
        remoteControlValues.Throttle = power;
    }

    virtual void RunControlLoop()
    {
        // Read sensor data
        Coordinates acc = sensors->GetAccelerationDirection();
        Coordinates gyro = sensors->GetGyroscopeDirection();
        Coordinates mag = sensors->GetCalibratedMagneticField();

        // Update IMU control values using sensor fusion
        imuControlValues = MathFunctions::ComputeFilter(acc, mag, &acc, &mag, &gyro);

        // Apply base throttle
        ApplyThrottle();

        // Calculate and apply PID corrections
        ControlErrors errors = GetControlErrors();
        ApplyPitch(errors.ControlPitch);
        ApplyRoll(errors.ControlRoll);
        ApplyYaw(errors.ControlYaw);

        // Clamp motor values to valid range (0-255)
        for (int i = 0; i < ConstConfig::MOTORS_COUNT; i++)
        {
            if (motorThrottles[i] < 0)
                motorThrottles[i] = 0;
            if (motorThrottles[i] > 255)
                motorThrottles[i] = 255;
        }

        // Send to motors
        motors->getMotor(0)->ChangeThrottle((uint8_t)motorThrottles[0]);
        motors->getMotor(1)->ChangeThrottle((uint8_t)motorThrottles[1]);
        motors->getMotor(2)->ChangeThrottle((uint8_t)motorThrottles[2]);
        motors->getMotor(3)->ChangeThrottle((uint8_t)motorThrottles[3]);
    }

    float CalculateGravityCompensation(float pitch, float roll)
    {
        // Gravity compensation scaled to throttle range (0-255)
        // Assumes level flight needs ~50% throttle, compensate for tilt
        float tiltFactor = 1.0 / (cos(pitch * ConstConfig::Pi / 180.0) * cos(roll * ConstConfig::Pi / 180.0));
        return (tiltFactor - 1.0) * remoteControlValues.Throttle;
    }

protected:
    void ApplyPitch(float controlPitch)
    {
        short throttle = (short)controlPitch;
        motorThrottles[0] += throttle;
        motorThrottles[1] += throttle;
        motorThrottles[2] -= throttle;
        motorThrottles[3] -= throttle;
    }

    void ApplyRoll(float controlRoll)
    {
        short throttle = (short)controlRoll;
        motorThrottles[0] += throttle;
        motorThrottles[1] -= throttle;
        motorThrottles[2] -= throttle;
        motorThrottles[3] += throttle;
    }

    void ApplyYaw(float controlYaw)
    {
        short throttle = (short)controlYaw;
        motorThrottles[0] += throttle;
        motorThrottles[1] -= throttle;
        motorThrottles[2] += throttle;
        motorThrottles[3] -= throttle;
    }

    void ApplyThrottle()
    {
        float mgFactor = CalculateGravityCompensation(imuControlValues.Pitch, imuControlValues.Roll);
        float throttle = remoteControlValues.Throttle + mgFactor;

        motorThrottles[0] = throttle;
        motorThrottles[1] = throttle;
        motorThrottles[2] = throttle;
        motorThrottles[3] = throttle;
    }

    virtual ControlErrors GetControlErrors() = 0;
    
    virtual ~BaseQuadCopterControl() {}

    Sensors *sensors;
    MultiMotorControl *motors;
    ControlModel imuControlValues;
    ControlModel remoteControlValues;
    PIDModel *rollPid;
    PIDModel *pitchPid;
    PIDModel *yawPid;
    float *motorThrottles;
};
#endif
