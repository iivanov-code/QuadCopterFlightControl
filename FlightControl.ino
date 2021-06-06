#include "Sensors.h"
#include "MultiMotorControl.h"
#include "ConstConfig.h"
#include "BLECommunicator.h"

Sensors *sensors;
BLECommunicator *bleCommunicator;
MultiMotorControl *motors;

void setup()
{

  // communicator = new BluetoothCommunicator();
  uint8_t *motorPins = new uint8_t[4];
  motorPins[0] = D2;
  motorPins[1] = D3;
  motorPins[2] = D4;
  motorPins[3] = D5;

  sensors = new Sensors();
  if (sensors->InitializeAll())
  {
    motors = new MultiMotorControl(ConstConfig::MOTORS_COUNT, motorPins, false);
    bleCommunicator = new BLECommunicator(sensors, motors);
  }
}

void loop()
{
  bleCommunicator->listenForConnections();
  delay(1000);
}
