#ifndef ADAPTIVE_QUAD_COPTER_CONTROL
#define ADAPTIVE_QUAD_COPTER_CONTROL

#include "../../Abstractions/BaseQuadCopterControl.h"
#include "../../Models/AdaptivePIDModel.h"
#include "../../Constants/ConstConfig.h"
#include "../Sensors.h"

class AdaptiveQuadCopterControl : public BaseQuadCopterControl
{
public:
    AdaptiveQuadCopterControl(uint8_t *motorPins);
    ~AdaptiveQuadCopterControl();
    void RunControlLoop() override;

protected:
    ControlErrors GetControlErrors() override;

private:
    AdaptivePIDModel *rollAdaptive;
    AdaptivePIDModel *pitchAdaptive;
    AdaptivePIDModel *yawAdaptive;
    unsigned long adaptationInterval;
    unsigned long lastAdaptTime;

    // Adaptive PID calculation
    float CalculateAdaptivePID(float input, float target, AdaptivePIDModel *pid);

    // Track error statistics for adaptation
    void TrackError(AdaptivePIDModel *pid, float error);

    // Adapt PID gains based on performance
    void AdaptGains(AdaptivePIDModel *pid, const char *axisName);
};

#endif
