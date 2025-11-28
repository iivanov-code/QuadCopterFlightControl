#ifndef BASE_THROTTLE_CONTROL
#define BASE_THROTTLE_CONTROL

#include "Arduino.h"
using namespace std;

class BaseThrottleControl
{
public:
    BaseThrottleControl(uint8_t _pin, bool _bidirectional = false)
    {
        pin = _pin;
        bidirectional = _bidirectional;
    }

    virtual uint8_t SetThrottlePercent(float percentage) = 0;
    virtual void Arm() = 0;
    virtual void Disarm() = 0;

    virtual uint8_t GetThrottle()
    {
        return currentThrottle;
    }

    virtual int16_t GetMinThrottle()
    {
        return MinThrottle;
    }

    virtual int16_t GetMaxThrottle()
    {
        return MaxThrottle;
    }

protected:
    int16_t MinThrottle;
    int16_t MaxThrottle;
    virtual void SetThrottle(uint8_t throttle) = 0;
    uint8_t currentThrottle = 0;
    byte pin;
    bool bidirectional = false;
};

#endif
