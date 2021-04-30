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
    Coordinates getAccelerationDirection();
    Coordinates getGyroscopeDirection();
    Coordinates getMagneticField();
    Coordinates getCalibratedMagneticField();
    void CalibrateMagnetoMeter();
    bool initializeBarometricSensor();
    bool initializeIMUSensor();
    bool initializeTempAndHumiditySensor();
    bool initializeAll();

private:
    float CalculateNormalizedValue(float value, float min, float max);
    Coordinates acceleration;
    Coordinates gyroscope;
    Coordinates magneticField;
    Coordinates magneticFieldMaxValues;
    Coordinates magneticFieldMinValues;
};

#endif
