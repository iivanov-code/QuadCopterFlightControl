#ifndef PID_MODEL
#define PID_MODEL
struct PIDModel
{
    float Proportional;
    float Integral;
    float Derivative;
    float PrevError;
    float IntegralError;
}

#endif