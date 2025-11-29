#ifndef THROTTLE_CONTROL
#define THROTTLE_CONTROL

#include "../Abstractions/BaseThrottleControl.h"
#include <Servo.h>
using namespace std;

class ThrottleControl : public BaseThrottleControl
{
public:
    ThrottleControl(uint8_t _pin, bool _bidirectional = false)
        : BaseThrottleControl(_pin, _bidirectional)
    {
        MinThrottle = 1000;
        MaxThrottle = 2000;
        esc.attach(pin);
    }

    uint8_t SetThrottlePercent(float percentage) override;
    void Arm() override;
    void Disarm() override;

protected:
    void SetThrottle(uint8_t throttle) override;

private:
    Servo esc;
};

#endif