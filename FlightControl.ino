#include "Sensors.h"
#include "BluetoothCommunicator.h"

Sensors *sensors;
BluetoothCommunicator *communicator;
bool sensorsInitilized = false;

void setup()
{
  Serial.begin(9600);
  sensors = new Sensors();
  communicator = new BluetoothCommunicator();
  sensorsInitilized = sensors->InitializeAll();
}

void loop()
{
}
