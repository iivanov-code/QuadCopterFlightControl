#include "MathFunctions.h"

//O - pitch
float MathFunctions::GetPitchAngle(Coordinates acc)
{
    return atan(acc.X / sqrt(pow(acc.Y, 2) + pow(acc.Z, 2))) * ConstConfig::RADIANS;
}

//Ф - roll
float MathFunctions::GetRollAngle(Coordinates acc)
{
    return atan2(acc.Y, acc.Z) * ConstConfig::RADIANS;
}

float MathFunctions::GetYawAngle(Coordinates mag)
{
    return atan2(mag.Y, mag.X);
}

float MathFunctions::GetYawAngleCompensation(Coordinates mag, float pitch, float roll)
{
    return tan(-1 * (mag.Z * sin(roll) - mag.Y * cos(roll)) / (mag.X * cos(pitch) + mag.Y * sin(pitch) * sin(roll) + mag.Z * sin(pitch) * cos(roll)));
}

ControlModel MathFunctions::ComputeFilter(Coordinates rawAcc, Coordinates rawMag, Coordinates *acc, Coordinates *mag, Coordinates *gyro)
{
    ControlModel values;
    float accelerationMagnitude;
    float pitchAcceleration;
    float rollAcceleration;
    float yawMag;

    //Acceleration low pass filter
    acc->Y = acc->Y + ConstConfig::FILTERS_COEFICIENT * (rawAcc.Y - acc->Y);
    acc->X = acc->X + ConstConfig::FILTERS_COEFICIENT * (rawAcc.X - acc->X);
    acc->Z = acc->Z + ConstConfig::FILTERS_COEFICIENT * (rawAcc.Z - acc->Z);

    //Apply Gyro Integral
    values.Pitch += gyro->X * ConstConfig::DELTA_T;
    values.Roll += gyro->Y * ConstConfig::DELTA_T;
    values.Yaw += gyro->Z * ConstConfig::DELTA_T;

    //Should be near 1g
    accelerationMagnitude = abs(acc->X) + abs(acc->Y) + abs(acc->Z);

    //If not experiencing external acceleration
    if (abs(accelerationMagnitude - 1) < 0.2)
    {
        //Apply Trigger
        pitchAcceleration = ConstConfig::RADIANS * atan2(-acc->Y, acc->Z);
        rollAcceleration = ConstConfig::RADIANS * atan2(acc->X, sqrt(pow(acc->Y, 2) + pow(acc->Z, 2)));

        //Combine Gyro and Accelerometer Data
        values.Pitch = ConstConfig::GYRO_COEFICIENT * values.Pitch + ConstConfig::ACC_COEFICIENT * pitchAcceleration;
        values.Roll = ConstConfig::GYRO_COEFICIENT * values.Roll + ConstConfig::ACC_COEFICIENT * rollAcceleration;
    }

    //Magnetometer Low Pass Filter
    mag->X = mag->X + ConstConfig::FILTERS_COEFICIENT * (rawMag.X - mag->X);
    mag->Y = mag->Y + ConstConfig::FILTERS_COEFICIENT * (rawMag.Y - mag->Y);
    mag->Z = mag->Z + ConstConfig::FILTERS_COEFICIENT * (rawMag.Z - mag->Z);

    //Should be near 1
    float magMagnitude = abs(mag->X) + abs(mag->Y) + abs(mag->Z);

    //If not experiencing external fields
    if (abs(magMagnitude - 1) < 0.2)
    {
        float num = mag->Z * sin(values.Roll) - mag->Y * cos(values.Roll);
        float den = mag->X * cos(values.Pitch) + mag->Y * sin(values.Pitch) * sin(values.Roll) + mag->Z * sin(values.Pitch) * cos(values.Roll);
        yawMag = ConstConfig::RADIANS * atan2(num, den); //Apply Trig
        values.Yaw = ConstConfig::GYRO_COEFICIENT * values.Yaw + ConstConfig::ACC_COEFICIENT * yawMag;
    }

    return values;
}

float MathFunctions::CalculatePID(float input, float target, PIDModel *pid)
{
    float error = target - input; //Calculate Setpoint Error

    //Proportional
    float pTerm = pid->Proportional * error;

    //Integral (accumulate error)
    pid->IntegralError += error * ConstConfig::DELTA_T;

    //Derivative
    float dTerm = pid->Derivative * (error - pid->PrevError) / ConstConfig::DELTA_T;

    //For next derivative term
    pid->PrevError = error;

    //Anti-windup: clamp integral error
    if (pid->IntegralError > ConstConfig::INTEGRAL_THRESHOLD)
    {
        pid->IntegralError = ConstConfig::INTEGRAL_THRESHOLD;
    }
    else if (pid->IntegralError < -ConstConfig::INTEGRAL_THRESHOLD)
    {
        pid->IntegralError = -ConstConfig::INTEGRAL_THRESHOLD;
    }

    //Calculate integral term
    float iTerm = pid->Integral * pid->IntegralError;

    return pTerm + iTerm + dTerm;
}