#ifndef SENSORS
#define SENSORS

#include <Arduino_LSM9DS1.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>
#include "Coordinates.h"

class Sensors
{
public:
    Sensors();
    int getTemperature();
    float getPressure();
    int getHumidity();
    int getDirectionDegrees();
    Coordinates getAccelerationDirection();
    Coordinates getGyroscopeDirection();
    Coordinates getMagneticField();
    bool initializeBarometricSensor();
    bool initializeIMUSensor();
    bool initializeTempAndHumiditySensor();
    void initializeAll();

private:
    Coordinates acceleration;
    Coordinates gyroscope;
    Coordinates magneticField;
};

#endif