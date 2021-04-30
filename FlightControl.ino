#include "Sensors.h"
#include "String.h"

Sensors *sensors;
bool sensorsInitilized = false;

Coordinates gyroCoordinates;
Coordinates magneticFieldCoordinates;
Coordinates accelerometedCoordinates;

void setup()
{
  Serial.begin(9600);
  sensors = new Sensors();
  sensorsInitilized = sensors->initializeAll();
}


void PrintCoordinates(String sensorName, Coordinates coordinates)
{
  Serial.print(sensorName);
  Serial.print(" X: ");
  Serial.print(coordinates.X);
  Serial.print(" Y: ");
  Serial.print(coordinates.Y);
  Serial.print(" Z: ");
  Serial.println(coordinates.Z);
}

void loop()
{
  if (Serial)
  {
    if (sensorsInitilized)
    {
      gyroCoordinates = sensors->getGyroscopeDirection();
      magneticFieldCoordinates = sensors->getMagneticField();
      accelerometedCoordinates = sensors->getAccelerationDirection();
    }

    PrintCoordinates("Gyro", gyroCoordinates);
    PrintCoordinates("Magneto", magneticFieldCoordinates);
    PrintCoordinates("Accelerometer", accelerometedCoordinates);
  }

  delay(5000);
}
