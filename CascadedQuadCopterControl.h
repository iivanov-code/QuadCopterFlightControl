#ifndef CASCADED_QUAD_COPTER_CONTROL
#define CASCADED_QUAD_COPTER_CONTROL

#include "BaseQuadCopterControl.cpp"
#include "CascadedPIDModel.h"

class CascadedQuadCopterControl : public BaseQuadCopterControl
{
public:
    CascadedQuadCopterControl(uint8_t *motorPins) : BaseQuadCopterControl(motorPins)
    {
        // Replace single PIDs with cascaded PIDs
        delete rollPid;
        delete pitchPid;
        delete yawPid;
        
        rollCascaded = new CascadedPIDModel();
        pitchCascaded = new CascadedPIDModel();
        yawCascaded = new CascadedPIDModel();
    }
    
    ~CascadedQuadCopterControl()
    {
        delete rollCascaded;
        delete pitchCascaded;
        delete yawCascaded;
    }
    
    void RunControlLoop() override
    {
        // Read sensor data
        Coordinates acc = sensors->GetAccelerationDirection();
        Coordinates gyro = sensors->GetGyroscopeDirection();
        Coordinates mag = sensors->GetCalibratedMagneticField();
        
        // Store gyro rates for inner loop
        gyroRates = gyro;

        // Update IMU control values using sensor fusion
        imuControlValues = MathFunctions::ComputeFilter(acc, mag, &acc, &mag, &gyro);

        // Apply base throttle
        ApplyThrottle();

        // Calculate and apply cascaded PID corrections
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
        motors->getMotor(0)->SetThrottlePercent((uint8_t)motorThrottles[0]);
        motors->getMotor(1)->SetThrottlePercent((uint8_t)motorThrottles[1]);
        motors->getMotor(2)->SetThrottlePercent((uint8_t)motorThrottles[2]);
        motors->getMotor(3)->SetThrottlePercent((uint8_t)motorThrottles[3]);
    }

protected:
    ControlErrors GetControlErrors() override
    {
        // Cascaded PID: Outer loop (angle) -> Inner loop (rate)
        
        // ROLL: Angle error -> desired rate -> control output
        float rollDesiredRate = MathFunctions::CalculatePID(
            imuControlValues.Roll, 
            remoteControlValues.Roll, 
            rollCascaded->AnglePID
        );
        float controlRoll = MathFunctions::CalculatePID(
            gyroRates.X, 
            rollDesiredRate, 
            rollCascaded->RatePID
        );
        
        // PITCH: Angle error -> desired rate -> control output
        float pitchDesiredRate = MathFunctions::CalculatePID(
            imuControlValues.Pitch, 
            remoteControlValues.Pitch, 
            pitchCascaded->AnglePID
        );
        float controlPitch = MathFunctions::CalculatePID(
            gyroRates.Y, 
            pitchDesiredRate, 
            pitchCascaded->RatePID
        );
        
        // YAW: Angle error -> desired rate -> control output
        float yawDesiredRate = MathFunctions::CalculatePID(
            imuControlValues.Yaw, 
            remoteControlValues.Yaw, 
            yawCascaded->AnglePID
        );
        float controlYaw = MathFunctions::CalculatePID(
            gyroRates.Z, 
            yawDesiredRate, 
            yawCascaded->RatePID
        );

        ControlErrors errors;
        errors.ControlRoll = controlRoll;
        errors.ControlPitch = controlPitch;
        errors.ControlYaw = controlYaw;

        return errors;
    }

private:
    CascadedPIDModel *rollCascaded;
    CascadedPIDModel *pitchCascaded;
    CascadedPIDModel *yawCascaded;
    Coordinates gyroRates; // Store gyro rates for inner loop
};

#endif
