#ifndef MATH_FUNCTIONS
#define MATH_FUNCTIONS

#include <math.h>
#include <fastmath.h>
#include "Coordinates.h"
#include "ControlModel.h"
#include "PIDModel.h"
#include "ConstConfig.h"

class MathFunctions
{
public:
    //Get Pitch angle from accelerometer
    static float GetPitchAngle(Coordinates acc);
    //Get Roll angle from accelerometer
    static float GetRollAngle(Coordinates acc);
    //Get YAW angle from magnetometer
    static float GetYawAngle(Coordinates mag);
    //Get Roll angle from magnetometer compensated with ...
    static float GetYawAngleCompensation(Coordinates mag, float pitch, float roll);
    static ControlModel ComputeFilter(Coordinates rawAcc, Coordinates rawMag, Coordinates *acc, Coordinates *mag, Coordinates *gyro);
    static float CalculatePID(float input, float target, PIDModel *pid);

private:
    MathFunctions();
};
#endif