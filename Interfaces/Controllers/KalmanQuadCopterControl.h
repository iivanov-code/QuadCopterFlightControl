#ifndef KALMAN_QUAD_COPTER_CONTROL
#define KALMAN_QUAD_COPTER_CONTROL

#include "../../Abstractions/BaseQuadCopterControl.h"
#include "../../Models/CascadedPIDModel.h"
#include "../../Constants/ConstConfig.h"
#include "../KalmanFilter.h"
#include "../Sensors.h"

class KalmanQuadCopterControl : public BaseQuadCopterControl
{
public:
    KalmanQuadCopterControl(uint8_t *motorPins);
    ~KalmanQuadCopterControl();
    void RunControlLoop() override;

protected:
    ControlErrors GetControlErrors() override;

private:
    KalmanFilter *rollKalman;
    KalmanFilter *pitchKalman;
    KalmanFilter *yawKalman;
    unsigned long lastUpdateTime;
    bool initialized;
};

#endif
