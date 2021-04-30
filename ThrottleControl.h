#ifndef THROTTLE_CONTROL
#define THROTTLE_CONTROL

#include "Arduino.h"
using namespace std;

class ThrottleControl
{

public:
    ThrottleControl(uint8_t _pin, bool _bidirectional = false);
    uint8_t Up(short percentage);
    uint8_t Down(short percentage);
    uint8_t GetThrottle();
private:
    uint8_t ConvertToThrottle(short percentage);
    uint8_t currentThrottle = 0;
    byte pin;
    bool bidirectional = false;
    uint8_t ChangeThrottle(short percentage);
};

#endif
