#ifndef QUAD_COPTER_CONTROL
#define QUAD_COPTER_CONTROL

#include "BaseQuadCopterControl.cpp"

class QuadCopterControl : public BaseQuadCopterControl
{
public:
    QuadCopterControl(uint8_t *motorPins);
    ~QuadCopterControl();

protected:
    ControlErrors GetControlErrors() override;
};
#endif
