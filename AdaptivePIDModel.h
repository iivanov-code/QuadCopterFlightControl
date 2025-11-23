#ifndef ADAPTIVE_PID_MODEL
#define ADAPTIVE_PID_MODEL

#include "PIDModel.h"

struct AdaptivePIDModel
{
    AdaptivePIDModel()
    {
        // Initial PID gains (conservative defaults)
        Proportional = 5.0;
        Integral = 0.5;
        Derivative = 10.0;
        
        // Adaptive parameters
        PrevError = 0;
        IntegralError = 0;
        ErrorSum = 0;
        ErrorCount = 0;
        
        // Auto-tuning parameters
        MaxError = 0;
        MinError = 0;
        SettleTime = 0;
        LastAdaptTime = 0;
        
        // Gain adjustment bounds
        KpMin = 1.0;
        KpMax = 15.0;
        KiMin = 0.1;
        KiMax = 3.0;
        KdMin = 5.0;
        KdMax = 30.0;
        
        // Adaptation rate (how aggressively to adjust gains)
        AdaptationRate = 0.1;
        
        // Performance thresholds
        ErrorThresholdHigh = 15.0;  // Large error - increase gains
        ErrorThresholdLow = 2.0;    // Small error - fine tune
        OscillationThreshold = 5.0; // Detect oscillation
    }

    // Current PID gains (adaptive)
    float Proportional;
    float Integral;
    float Derivative;
    
    // Error tracking
    float PrevError;
    float IntegralError;
    float ErrorSum;
    int ErrorCount;
    
    // Performance metrics
    float MaxError;
    float MinError;
    float SettleTime;
    unsigned long LastAdaptTime;
    
    // Gain bounds
    float KpMin, KpMax;
    float KiMin, KiMax;
    float KdMin, KdMax;
    
    // Adaptation parameters
    float AdaptationRate;
    float ErrorThresholdHigh;
    float ErrorThresholdLow;
    float OscillationThreshold;
    
    // Oscillation detection
    int SignChanges = 0;
    float LastErrorSign = 0;
};

#endif
