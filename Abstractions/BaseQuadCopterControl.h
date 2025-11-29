#ifndef BASE_QUAD_COPTER_CONTROL
#define BASE_QUAD_COPTER_CONTROL

#include "../Interfaces/Sensors.h"
#include "../Interfaces/MultiMotorControl.h"
#include "../Models/QuadCopterTypes.h"
#include "../Models/Coordinates.h"
#include "../Models/ControlModel.h"
#include "../Models/PIDModel.h"
#include "../Models/ControlErrors.h"
#include "../Interfaces/MathFunctions.h"
#include "../Constants/ConstConfig.h"

const uint8_t MIN_PERCENT = 0;
const uint8_t MAX_PERCENT = 100;

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

    virtual ~BaseQuadCopterControl()
    {
        delete motors;
        delete sensors;
        delete rollPid;
        delete pitchPid;
        delete yawPid;
        delete[] motorThrottles;
    }

     // Or strongly-typed (C++11+)
    enum class PercentageLimits : uint8_t {
        Min = 0,
        Max = 100
    };

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
            if (motorThrottles[i] < MIN_PERCENT)
                motorThrottles[i] = MIN_PERCENT;
            if (motorThrottles[i] > MAX_PERCENT)
                motorThrottles[i] = MAX_PERCENT ;
        }

        // Send to motors
        motors->getMotor(0)->SetThrottlePercent((uint8_t)motorThrottles[0]);
        motors->getMotor(1)->SetThrottlePercent((uint8_t)motorThrottles[1]);
        motors->getMotor(2)->SetThrottlePercent((uint8_t)motorThrottles[2]);
        motors->getMotor(3)->SetThrottlePercent((uint8_t)motorThrottles[3]);
    }

    virtual float CalculateGravityCompensation(float pitch, float roll)
    {
        // Gravity compensation scaled to throttle range (0-100)
        // Assumes level flight needs ~50% throttle, compensate for tilt
        float tiltFactor = 1.0 / (cos(pitch * ConstConfig::Pi / 180.0) * cos(roll * ConstConfig::Pi / 180.0));
        return (tiltFactor - 1.0) * remoteControlValues.Throttle;
    }

protected:
    virtual void ApplyPitch(float controlPitch)
    {
        short throttle = (short)controlPitch;
        motorThrottles[0] += throttle;
        motorThrottles[1] += throttle;
        motorThrottles[2] -= throttle;
        motorThrottles[3] -= throttle;
    }

    virtual void ApplyRoll(float controlRoll)
    {
        short throttle = (short)controlRoll;
        motorThrottles[0] += throttle;
        motorThrottles[1] -= throttle;
        motorThrottles[2] -= throttle;
        motorThrottles[3] += throttle;
    }

    virtual void ApplyYaw(float controlYaw)
    {
        short throttle = (short)controlYaw;
        motorThrottles[0] += throttle;
        motorThrottles[1] -= throttle;
        motorThrottles[2] += throttle;
        motorThrottles[3] -= throttle;
    }

    virtual void ApplyThrottle()
    {
        float mgFactor = CalculateGravityCompensation(imuControlValues.Pitch, imuControlValues.Roll);
        float throttle = remoteControlValues.Throttle + mgFactor;

        motorThrottles[0] = throttle;
        motorThrottles[1] = throttle;
        motorThrottles[2] = throttle;
        motorThrottles[3] = throttle;
    }

    virtual ControlErrors GetControlErrors() = 0;

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
