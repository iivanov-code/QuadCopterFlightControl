#ifndef THROTTLE_CONTROL
#define THROTTLE_CONTROL

#include "../Abstractions/BaseThrottleControl.h"
#include <Servo.h>
using namespace std;

class ThrottleControl : public BaseThrottleControl
{
public:
    ThrottleControl(uint8_t _pin, bool _bidirectional = false);

private:
    Servo esc;
};

#endif