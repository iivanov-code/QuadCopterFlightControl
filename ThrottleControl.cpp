#include "ThrottleControl.h"

using namespace std;

const uint8_t MAX_THROTTLE = 255u;
const uint8_t MAX_PERCENTAGE = 100u;

ThrottleControl::ThrottleControl(uint8_t _pin, bool _bidirectional)
{
  pin = _pin;
  currentThrottle = 0u;
  bidirectional = _bidirectional;
  pinMode(pin, OUTPUT);
}

uint8_t ThrottleControl::ChangeThrottle(uint8_t throttle)
{
  currentThrottle = throttle;
  analogWrite(pin, currentThrottle);
  return currentThrottle;
}

uint8_t ThrottleControl::ConvertToThrottle(float percentage)
{
  if (bidirectional)
  {
    if (percentage < 0)
    {
      return ((MAX_THROTTLE / 2) * percentage) / MAX_PERCENTAGE;
    }
    else if (percentage > 0)
    {
      uint8_t halfThrottle = (MAX_THROTTLE / 2);
      return halfThrottle + ((halfThrottle * percentage) / MAX_PERCENTAGE);
    }

    return 0;
  }
  else
  {
    if (percentage < 0)
    {
      percentage *= -1;
    }

    return (MAX_THROTTLE * percentage) / MAX_PERCENTAGE;
  }
}
