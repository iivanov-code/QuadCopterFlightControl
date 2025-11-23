#ifndef PID_MODEL
#define PID_MODEL

struct PIDModel
{
    PIDModel()
    {
        Proportional = 8;
        Integral = 1;
        Derivative = 20;
        PrevError = 0;
        IntegralError = 0;
    }

    float Proportional;
    float Integral;
    float Derivative;
    float PrevError;
    float IntegralError;
};

#endif