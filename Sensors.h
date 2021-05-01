#ifndef SENSORS
#define SENSORS

#include <Arduino_LSM9DS1.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_HTS221.h>
#include "Coordinates.h"

class Sensors
{
public:
    int GetTemperature();
    float GetPressure();
    int GetHumidity();
    Coordinates GetAccelerationDirection();
    Coordinates GetGyroscopeDirection();
    Coordinates GetMagneticField();
    Coordinates GetCalibratedMagneticField();
    void CalibrateMagnetoMeter();
    bool InitializeBarometricSensor();
    bool InitializeIMUSensor();
    bool InitializeTempAndHumiditySensor();
    bool InitializeAll();

private:
    float CalculateNormalizedValue(float value, float min, float max);
    Coordinates acceleration;
    Coordinates gyroscope;
    Coordinates magneticField;
    Coordinates magneticFieldMaxValues;
    Coordinates magneticFieldMinValues;
};

#endif
