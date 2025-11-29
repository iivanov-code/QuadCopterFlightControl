#include "../Interfaces/ThrottleControl.h"

void ThrottleControl::Arm()
{
    // Arm ESC by sending minimum pulse
    esc.writeMicroseconds(MinThrottle);
}

void ThrottleControl::Disarm()
{
    // Disarm ESC by sending zero pulse
    esc.detach();
}

uint8_t ThrottleControl::SetThrottlePercent(float percentage)
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

void ThrottleControl::SetThrottle(uint8_t throttle)
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