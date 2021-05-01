#include "MathFunctions.h"

const float ACC_COEFICIENT = 0.01;
const float GYRO_COEFICIENT = 0.99;
const float DELTA_T = 0.002;
const float INTEGRAL_THRESHOLD = 200;
const float FILTERS_COEFICIENT = 0.1;

//O - pitch
float MathFunctions::GetPitchAngle(Coordinates acc)
{
    return atan(acc.X / sqrt(pow(acc.Y, 2) + pow(acc.Z, 2))) * (180 / PI);
}

//Ф - roll
float MathFunctions::GetRollAngle(Coordinates acc)
{
    return atan2(acc.Y, acc.Z) * (180 / PI);
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
    acc->Y = acc->Y + FILTERS_COEFICIENT * (rawAcc.Y - acc->Y);
    acc->X = acc->X + FILTERS_COEFICIENT * (rawAcc.X - acc->X);
    acc->Z = acc->Z + FILTERS_COEFICIENT * (rawAcc.Z - acc->Z);

    //Apply Gyro Integral
    values.Pitch += gyro->X * DELTA_T;
    values.Roll += gyro->Y * DELTA_T;
    values.Yaw += gyro->Z * DELTA_T;

    //Should be near 1g
    accelerationMagnitude = abs(acc->X) + abs(acc->Y) + abs(acc->Z);

    //If not experiencing external acceleration
    if (abs(accelerationMagnitude - 1) < 0.2)
    {
        //Apply Trigger
        pitchAcceleration = (180 / PI) * atan2(-acc->Y, acc->Z);
        rollAcceleration = (180 / PI) * atan2(acc->X, sqrt(pow(acc->Y, 2) + pow(acc->Z, 2)));

        //Combine Gyro and Accelerometer Data
        values.Pitch = GYRO_COEFICIENT * values.Pitch + ACC_COEFICIENT * pitchAcceleration;
        values.Roll = GYRO_COEFICIENT * values.Roll + ACC_COEFICIENT * rollAcceleration;
    }

    //Magnetometer Low Pass Filter
    mag->X = mag->X + FILTERS_COEFICIENT * (rawMag.X - mag->X);
    mag->Y = mag->Y + FILTERS_COEFICIENT * (rawMag.Y - mag->Y);
    mag->Z = mag->Z + FILTERS_COEFICIENT * (rawMag.Z - mag->Z);

    //Should be near 1
    float magMagnitude = abs(mag->X) + abs(mag->Y) + abs(mag->Z);

    //If not experiencing external fields
    if (abs(magMagnitude - 1) < 0.2)
    {
        float num = mag->Z * sin(values.Roll) - mag->Y * cos(values.Roll);
        float den = mag->X * cos(1) + mag->Y * sin(values.Pitch) * sin(values.Roll) + mag->Z * sin(values.Pitch) * cos(values.Roll);
        yawMag = (180 / PI) * atan2(num, den); //Apply Trig
        values.Yaw = GYRO_COEFICIENT * values.Yaw + ACC_COEFICIENT * yawMag;
    }

    return values;
}

float MathFunctions::CalculatePID(float input, float target, PIDModel *pid)
{
    float error = target - input; //Calculate Setpoint Error
    
    //Proportional
    pid->Proportional = pid->Proportional * error;
    
    //Integral
    pid->Integral = pid->Integral * error * DELTA_T + pid->Integral;
    
    //Derivative
    pid->Derivative = pid->Derivative * (error - pid->PrevError) * DELTA_T;
    
    //For next derivative term
    pid->PrevError = error;

    //For next integral term
    pid->IntegralError += pid->Integral * error * DELTA_T;

    if (abs(pid->Integral) > INTEGRAL_THRESHOLD)
    {
        if (pid->IntegralError > 0)
        {
            pid->IntegralError = INTEGRAL_THRESHOLD;
        }
        else
        {
            pid->IntegralError = -INTEGRAL_THRESHOLD;
        }

        if (pid->Integral > 0)
        {
            pid->Integral = INTEGRAL_THRESHOLD;
        }
        else
        {
            pid->Integral = -INTEGRAL_THRESHOLD;
        }
    }

    return pid->Proportional + pid->Integral + pid->Derivative;
}