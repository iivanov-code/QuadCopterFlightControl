#ifndef MATH_FUNCTIONS
#define MATH_FUNCTIONS

#include <math.h>
#include <fastmath.h>
#include "Coordinates.h"
#include "ControlModel.h"
#include "PIDModel.h"

#define PI 3.1415926535897932384626433832795

class MathFunctions
{
public:
    float GetPitchAngle(Coordinates acc);
    float GetRollAngle(Coordinates acc);
    float GetYawAngle(Coordinates mag);
    float GetYawAngleCompensation(Coordinates mag, float pitch, float roll);
    
    ControlModel ComputeFilter(Coordinates rawAcc, Coordinates rawMag, Coordinates *acc, Coordinates *mag, Coordinates *gyro);
    float CalculatePID(float input, float target, PID *pid);
    

private:
    ControlModel controlValues;
};
#endif