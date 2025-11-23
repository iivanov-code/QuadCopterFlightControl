#include "QuadCopterControl.h"
#include "CascadedQuadCopterControl.h"
#include "AdaptiveQuadCopterControl.h"
#include "KalmanQuadCopterControl.h"
#include "KalmanCascadedQuadCopterControl.h"
#include "ConstConfig.h"

BaseQuadCopterControl *quadControl;
enum ControllerMode { SINGLE, CASCADED, ADAPTIVE, KALMAN, KALMAN_CASCADED };
ControllerMode currentMode = KALMAN_CASCADED; // Default to best performer
unsigned long lastPrintTime = 0;

void setup()
{
  Serial.begin(9600);
  while (!Serial);
  
  Serial.println("=========================================");
  Serial.println("  Quadcopter Flight Control System");
  Serial.println("=========================================");
  
  // Define motor pins
  uint8_t *motorPins = new uint8_t[4];
  motorPins[0] = D2;
  motorPins[1] = D3;
  motorPins[2] = D4;
  motorPins[3] = D5;

  // Initialize with selected controller
  if (currentMode == KALMAN_CASCADED)
  {
    Serial.println("\nUsing: KALMAN + CASCADED PID");
    Serial.println("  (Best: Optimal filtering + dual-loop control)");
    quadControl = new KalmanCascadedQuadCopterControl(motorPins);
  }
  else if (currentMode == KALMAN)
  {
    Serial.println("\nUsing: KALMAN + SINGLE PID");
    Serial.println("  (Better filtering, single-loop control)");
    quadControl = new KalmanQuadCopterControl(motorPins);
  }
  else if (currentMode == ADAPTIVE)
  {
    Serial.println("\nUsing: ADAPTIVE PID");
    Serial.println("  (Self-tuning gains)");
    quadControl = new AdaptiveQuadCopterControl(motorPins);
  }
  else if (currentMode == CASCADED)
  {
    Serial.println("\nUsing: CASCADED PID");
    Serial.println("  (Angle + Rate control)");
    quadControl = new CascadedQuadCopterControl(motorPins);
  }
  else
  {
    Serial.println("\nUsing: SINGLE PID");
    Serial.println("  (Basic control)");
    quadControl = new QuadCopterControl(motorPins);
  }
  
  quadControl->Initilize();
  
  Serial.println("\n✓ Quadcopter initialized!");
  Serial.println("\n--- Available Controllers ---");
  Serial.println("  '1' - Single PID");
  Serial.println("  '2' - Cascaded PID");
  Serial.println("  '3' - Adaptive PID");
  Serial.println("  '4' - Kalman + Single PID");
  Serial.println("  '5' - Kalman + Cascaded PID ★");
  Serial.println("=========================================\n");
  
  // Set initial targets for testing
  quadControl->Thrust(50);   // 50% throttle
  quadControl->Roll(0);      // Level roll
  quadControl->Pitch(0);     // Level pitch
  quadControl->Yaw(0);       // Hold yaw
}

void loop()
{
  // Check for controller switch command
  if (Serial.available() > 0)
  {
    char cmd = Serial.read();
    
    uint8_t *motorPins = new uint8_t[4];
    motorPins[0] = D2;
    motorPins[1] = D3;
    motorPins[2] = D4;
    motorPins[3] = D5;
    
    bool switched = false;
    
    if (cmd == '1')
    {
      if (currentMode != SINGLE)
      {
        Serial.println("\n>>> Switching to SINGLE PID <<<");
        delete quadControl;
        quadControl = new QuadCopterControl(motorPins);
        currentMode = SINGLE;
        switched = true;
      }
    }
    else if (cmd == '2')
    {
      if (currentMode != CASCADED)
      {
        Serial.println("\n>>> Switching to CASCADED PID <<<");
        delete quadControl;
        quadControl = new CascadedQuadCopterControl(motorPins);
        currentMode = CASCADED;
        switched = true;
      }
    }
    else if (cmd == '3')
    {
      if (currentMode != ADAPTIVE)
      {
        Serial.println("\n>>> Switching to ADAPTIVE PID <<<");
        delete quadControl;
        quadControl = new AdaptiveQuadCopterControl(motorPins);
        currentMode = ADAPTIVE;
        switched = true;
      }
    }
    else if (cmd == '4')
    {
      if (currentMode != KALMAN)
      {
        Serial.println("\n>>> Switching to KALMAN + SINGLE PID <<<");
        delete quadControl;
        quadControl = new KalmanQuadCopterControl(motorPins);
        currentMode = KALMAN;
        switched = true;
      }
    }
    else if (cmd == '5')
    {
      if (currentMode != KALMAN_CASCADED)
      {
        Serial.println("\n>>> Switching to KALMAN + CASCADED PID <<<");
        delete quadControl;
        quadControl = new KalmanCascadedQuadCopterControl(motorPins);
        currentMode = KALMAN_CASCADED;
        switched = true;
      }
    }
    
    if (switched)
    {
      quadControl->Initilize();
      quadControl->Thrust(50);
      quadControl->Roll(0);
      quadControl->Pitch(0);
      quadControl->Yaw(0);
      Serial.println("Ready!");
    }
    
    delete[] motorPins;
  }
  
  // Run the control loop
  quadControl->RunControlLoop();
  
  // Print status every 3 seconds
  if (millis() - lastPrintTime > 3000)
  {
    Serial.print("Active: ");
    switch(currentMode)
    {
      case SINGLE:
        Serial.println("Single PID");
        break;
      case CASCADED:
        Serial.println("Cascaded PID");
        break;
      case ADAPTIVE:
        Serial.println("Adaptive PID");
        break;
      case KALMAN:
        Serial.println("Kalman + Single PID");
        break;
      case KALMAN_CASCADED:
        Serial.println("Kalman + Cascaded PID ★");
        break;
    }
    
    lastPrintTime = millis();
  }
  
  // Control loop timing (500Hz = 2ms)
  delay(2);
}
