#ifndef CASCADED_QUAD_COPTER_CONTROL
#define CASCADED_QUAD_COPTER_CONTROL

#include "../../Abstractions/BaseQuadCopterControl.h"
#include "../../Models/CascadedPIDModel.h"
#include "../../Constants/ConstConfig.h"
#include "../Sensors.h"

class CascadedQuadCopterControl : public BaseQuadCopterControl
{
public:
    CascadedQuadCopterControl(uint8_t *motorPins);
    ~CascadedQuadCopterControl();
    void RunControlLoop() override;

protected:
    ControlErrors GetControlErrors() override;

private:
    CascadedPIDModel *rollCascaded;
    CascadedPIDModel *pitchCascaded;
    CascadedPIDModel *yawCascaded;
    Coordinates gyroRates; // Store gyro rates for inner loop
};

#endif
