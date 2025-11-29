#ifndef ADAPTIVE_QUAD_COPTER_CONTROL
#define ADAPTIVE_QUAD_COPTER_CONTROL

#include "../Interfaces/Controllers/AdaptiveQuadCopterControl.h"

AdaptiveQuadCopterControl::AdaptiveQuadCopterControl(uint8_t *motorPins)
    : BaseQuadCopterControl(motorPins)
{
    // Replace single PIDs with adaptive PIDs
    delete rollPid;
    delete pitchPid;
    delete yawPid;

    rollAdaptive = new AdaptivePIDModel();
    pitchAdaptive = new AdaptivePIDModel();
    yawAdaptive = new AdaptivePIDModel();

    adaptationInterval = 500; // Adapt every 500ms
    lastAdaptTime = millis();
}

AdaptiveQuadCopterControl::~AdaptiveQuadCopterControl()
{
    delete rollAdaptive;
    delete pitchAdaptive;
    delete yawAdaptive;
}

void AdaptiveQuadCopterControl::RunControlLoop() override
{
    // Read sensor data
    Coordinates acc = sensors->GetAccelerationDirection();
    Coordinates gyro = sensors->GetGyroscopeDirection();
    Coordinates mag = sensors->GetCalibratedMagneticField();

    // Update IMU control values using sensor fusion
    imuControlValues = MathFunctions::ComputeFilter(acc, mag, &acc, &mag, &gyro);

    // Periodically adapt PID gains based on performance
    if (millis() - lastAdaptTime > adaptationInterval)
    {
        AdaptGains(rollAdaptive, "Roll");
        AdaptGains(pitchAdaptive, "Pitch");
        AdaptGains(yawAdaptive, "Yaw");
        lastAdaptTime = millis();
    }

    // Apply base throttle
    ApplyThrottle();

    // Calculate and apply adaptive PID corrections
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
            motorThrottles[i] = MAX_PERCENT;
    }

    // Send to motors
    motors->getMotor(0)->SetThrottlePercent((uint8_t)motorThrottles[0]);
    motors->getMotor(1)->SetThrottlePercent((uint8_t)motorThrottles[1]);
    motors->getMotor(2)->SetThrottlePercent((uint8_t)motorThrottles[2]);
    motors->getMotor(3)->SetThrottlePercent((uint8_t)motorThrottles[3]);
}

ControlErrors AdaptiveQuadCopterControl::GetControlErrors() override
{
    // Calculate errors
    float rollError = remoteControlValues.Roll - imuControlValues.Roll;
    float pitchError = remoteControlValues.Pitch - imuControlValues.Pitch;
    float yawError = remoteControlValues.Yaw - imuControlValues.Yaw;

    // Track error statistics for adaptation
    TrackError(rollAdaptive, rollError);
    TrackError(pitchAdaptive, pitchError);
    TrackError(yawAdaptive, yawError);

    // Calculate adaptive PID outputs
    float controlRoll = CalculateAdaptivePID(imuControlValues.Roll, remoteControlValues.Roll, rollAdaptive);
    float controlPitch = CalculateAdaptivePID(imuControlValues.Pitch, remoteControlValues.Pitch, pitchAdaptive);
    float controlYaw = CalculateAdaptivePID(imuControlValues.Yaw, remoteControlValues.Yaw, yawAdaptive);

    ControlErrors errors;
    errors.ControlRoll = controlRoll;
    errors.ControlPitch = controlPitch;
    errors.ControlYaw = controlYaw;

    return errors;
}

// Adaptive PID calculation
float AdaptiveQuadCopterControl::CalculateAdaptivePID(float input, float target, AdaptivePIDModel *pid)
{
    float error = target - input;

    // Proportional term
    float pTerm = pid->Proportional * error;

    // Integral term with anti-windup
    pid->IntegralError += error * ConstConfig::DELTA_T;

    if (pid->IntegralError > ConstConfig::INTEGRAL_THRESHOLD)
        pid->IntegralError = ConstConfig::INTEGRAL_THRESHOLD;
    else if (pid->IntegralError < -ConstConfig::INTEGRAL_THRESHOLD)
        pid->IntegralError = -ConstConfig::INTEGRAL_THRESHOLD;

    float iTerm = pid->Integral * pid->IntegralError;

    // Derivative term
    float dTerm = pid->Derivative * (error - pid->PrevError) / ConstConfig::DELTA_T;

    pid->PrevError = error;

    return pTerm + iTerm + dTerm;
}

// Track error statistics for adaptation
void AdaptiveQuadCopterControl::TrackError(AdaptivePIDModel *pid, float error)
{
    float absError = abs(error);

    // Update error statistics
    pid->ErrorSum += absError;
    pid->ErrorCount++;

    if (absError > pid->MaxError)
        pid->MaxError = absError;

    // Detect oscillation (sign changes)
    float errorSign = (error > 0) ? 1.0 : -1.0;
    if (errorSign != pid->LastErrorSign && pid->LastErrorSign != 0)
    {
        pid->SignChanges++;
    }
    pid->LastErrorSign = errorSign;
}

// Adapt PID gains based on performance
void AdaptiveQuadCopterControl::AdaptGains(AdaptivePIDModel *pid, const char *axisName)
{
    if (pid->ErrorCount == 0)
        return;

    float avgError = pid->ErrorSum / pid->ErrorCount;
    float oscillationRate = (float)pid->SignChanges / pid->ErrorCount;

    // Determine adaptation strategy based on error characteristics

    // Case 1: Large error - increase proportional gain
    if (avgError > pid->ErrorThresholdHigh)
    {
        pid->Proportional += pid->AdaptationRate * pid->Proportional;
        if (pid->Proportional > pid->KpMax)
            pid->Proportional = pid->KpMax;

        Serial.print(axisName);
        Serial.print(" - Large error detected. Kp increased to: ");
        Serial.println(pid->Proportional);
    }

    // Case 2: Oscillation detected - reduce derivative and proportional
    else if (oscillationRate > 0.3) // More than 30% sign changes
    {
        pid->Proportional *= 0.9; // Reduce by 10%
        pid->Derivative *= 1.1;   // Increase damping

        if (pid->Proportional < pid->KpMin)
            pid->Proportional = pid->KpMin;
        if (pid->Derivative > pid->KdMax)
            pid->Derivative = pid->KdMax;

        Serial.print(axisName);
        Serial.print(" - Oscillation detected. Kp: ");
        Serial.print(pid->Proportional);
        Serial.print(", Kd: ");
        Serial.println(pid->Derivative);
    }

    // Case 3: Steady state error - increase integral
    else if (avgError > pid->ErrorThresholdLow && avgError < pid->ErrorThresholdHigh)
    {
        if (abs(pid->IntegralError) > pid->ErrorThresholdLow)
        {
            pid->Integral += pid->AdaptationRate * 0.5 * pid->Integral;
            if (pid->Integral > pid->KiMax)
                pid->Integral = pid->KiMax;

            Serial.print(axisName);
            Serial.print(" - Steady state error. Ki increased to: ");
            Serial.println(pid->Integral);
        }
    }

    // Case 4: Good performance - fine tune
    else if (avgError < pid->ErrorThresholdLow && oscillationRate < 0.1)
    {
        // Slightly increase responsiveness
        pid->Proportional += pid->AdaptationRate * 0.5;
        if (pid->Proportional > pid->KpMax)
            pid->Proportional = pid->KpMax;
    }

    // Reset statistics for next adaptation cycle
    pid->ErrorSum = 0;
    pid->ErrorCount = 0;
    pid->MaxError = 0;
    pid->SignChanges = 0;
}