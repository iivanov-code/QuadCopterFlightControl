#include "Interfaces/QuadCopterControl.h"
#include "Imports.h"
#include "Interfaces/Controllers/CascadedQuadCopterControl.h"
#include "Interfaces/Controllers/AdaptiveQuadCopterControl.h"
#include "Interfaces/Controllers/KalmanQuadCopterControl.h"
#include "Interfaces/Controllers/KalmanCascadedQuadCopterControl.h"
#include "Constants/ConstConfig.h"
#include "Enums/ControllerMode.h"

BaseQuadCopterControl *quadControl;
const ControllerMode currentMode = KALMAN_CASCADED; // Default to best performer


void setup()
{
  Serial.begin(9600);

  // Define motor pins
  uint8_t *motorPins = new uint8_t[4];
  motorPins[0] = D2;
  motorPins[1] = D3;
  motorPins[2] = D4;
  motorPins[3] = D5;

  switch (currentMode)
  {
  case KALMAN_CASCADED:
    quadControl = new KalmanCascadedQuadCopterControl(motorPins);
    break;
  case KALMAN:
    quadControl = new KalmanQuadCopterControl(motorPins);
    break;
  case ADAPTIVE:
    quadControl = new AdaptiveQuadCopterControl(motorPins);
    break;
  case CASCADED:
    quadControl = new CascadedQuadCopterControl(motorPins);
    break;
  case SINGLE:
    quadControl = new QuadCopterControl(motorPins);
    break;
  default:
    quadControl = new QuadCopterControl(motorPins);
    break;
  }

  quadControl->Initilize();

  // Set initial targets for testing
  quadControl->Thrust(10); // 10% throttle
  quadControl->Roll(0);    // Level roll
  quadControl->Pitch(0);   // Level pitch
  quadControl->Yaw(0);     // Hold yaw
}

void loop()
{
  // Run the control loop
  quadControl->RunControlLoop();

  // Control loop timing (500Hz = 2ms)
  delay(2);
}
