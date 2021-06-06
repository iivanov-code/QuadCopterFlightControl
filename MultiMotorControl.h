#ifndef MULTI_MOTOR_CONTROL
#define MULTI_MOTOR_CONTROL

#include "Arduino.h"
#include "ThrottleControl.h"

using namespace std;

class MultiMotorControl
{
public:
  MultiMotorControl(uint8_t motorsCount, uint8_t *pins, bool bidirectional = false);
  ThrottleControl *getMotor(uint8_t i);
  ThrottleControl *operator[](int);

private:
  ThrottleControl **motors;
};
#endif
