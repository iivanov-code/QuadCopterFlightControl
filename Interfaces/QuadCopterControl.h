#ifndef QUAD_COPTER_CONTROL
#define QUAD_COPTER_CONTROL

#include "../Abstractions/BaseQuadCopterControl.h"

class QuadCopterControl : public BaseQuadCopterControl
{
public:
    QuadCopterControl(uint8_t *motorPins);
    ~QuadCopterControl();

protected:
    ControlErrors GetControlErrors() override;
};
#endif
