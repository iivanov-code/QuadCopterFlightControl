#ifndef CASCADED_PID_MODEL
#define CASCADED_PID_MODEL

#include "PIDModel.h"

struct CascadedPIDModel
{
    CascadedPIDModel()
    {
        // Outer loop (angle/attitude control) - slower, smoother
        AnglePID = new PIDModel();
        AnglePID->Proportional = 4.5;
        AnglePID->Integral = 0.5;
        AnglePID->Derivative = 1.0;
        
        // Inner loop (rate control) - faster, more responsive
        RatePID = new PIDModel();
        RatePID->Proportional = 1.2;
        RatePID->Integral = 0.05;
        RatePID->Derivative = 0.01;
    }
    
    ~CascadedPIDModel()
    {
        delete AnglePID;
        delete RatePID;
    }

    PIDModel *AnglePID;  // Outer loop: controls angle
    PIDModel *RatePID;   // Inner loop: controls angular rate (gyro)
};

#endif
