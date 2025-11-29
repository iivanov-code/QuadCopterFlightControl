#ifndef KALMAN_CASCADED_QUAD_COPTER_CONTROL
#define KALMAN_CASCADED_QUAD_COPTER_CONTROL

#include "../../Abstractions/BaseQuadCopterControl.h"
#include "../../Models/CascadedPIDModel.h"
#include "../../Constants/ConstConfig.h"
#include "../KalmanFilter.h"
#include "../Sensors.h"

class KalmanCascadedQuadCopterControl : public BaseQuadCopterControl
{
public:
    KalmanCascadedQuadCopterControl(uint8_t *motorPins);
    ~KalmanCascadedQuadCopterControl();
    void RunControlLoop() override;

protected:
    ControlErrors GetControlErrors() override;

private:
    CascadedPIDModel *rollCascaded;
    CascadedPIDModel *pitchCascaded;
    CascadedPIDModel *yawCascaded;
    KalmanFilter *rollKalman;
    KalmanFilter *pitchKalman;
    KalmanFilter *yawKalman;
    Coordinates gyroRates;
    unsigned long lastUpdateTime;
    bool initialized;
};

#endif
