#include "MultiMotorControl.h"
//#include "Pins.h"

MultiMotorControl *motorsControler;

void setup()
{
  uint8_t *pins = new uint8_t[4];
  pins[0] = D2;
  pins[1] = D3;
  pins[2] = D4;
  pins[3] = D5;

  motorsControler = new MultiMotorControl(0x1, pins);
}

void loop()
{
  // put your main code here, to run repeatedly:
  short percentage = 3;
  short step = 10;

  ThrottleControl *control = motorsControler->getMotor(0u);
  control->Up(3);
  wait_ns(2000);
  control->Down(100);

  Serial.println("Start stepping UP");

  for (int i = 0; i < 10; i++)
  {
    percentage += step;
    control->Up(percentage);
    wait_ns(5000);
  }

  Serial.println("Start stepping DOWN");

  for (int i = 0; i < 10; i++)
  {
    percentage -= step;
    control->Down(percentage);
    wait_ns(5000);
  }

  Serial.println("Procedure finished!");
}
