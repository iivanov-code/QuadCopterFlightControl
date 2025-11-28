#ifndef THROTTLE_CONTROL
#define THROTTLE_CONTROL

#include "BaseThrottleControl.h"
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

    void Arm() override
    {
        // Arm ESC by sending minimum pulse
        esc.writeMicroseconds(MinThrottle);
    }

    void Disarm() override
    {
        // Disarm ESC by sending zero pulse
        esc.detach();
    }

    uint8_t SetThrottlePercent(float percentage) override
    {
        if (percentage < 0.0)
        {
            if (!bidirectional)
            {
                percentage = 0.0;
            }
            else if (percentage < -100.0)
            {
                percentage = -100.0;
            }
        }

        if (percentage > 100.0)
        {
            percentage = 100.0;
        }

        uint8_t throttleValue = (uint8_t)((float)MinThrottle + ((float)MaxThrottle - (float)MinThrottle) * (percentage / 100.0f));

        SetThrottle(throttleValue);

        return throttleValue;
    }

private:
    Servo esc;

protected:
    void SetThrottle(uint8_t throttle) override
    {
        if (throttle < MinThrottle)
        {
            throttle = MinThrottle;
        }

        if (throttle > MaxThrottle)
        {
            throttle = MaxThrottle;
        }

        esc.writeMicroseconds(throttle);
    }
};

#endif