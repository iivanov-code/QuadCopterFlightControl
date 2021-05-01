#include "Sensors.h"

Sensors::Sensors()
{

}

int Sensors::getTemperature()
{
    //CELSIUS
    //FAHRENHEIT
    return HTS.readTemperature(CELSIUS);
}

int Sensors::getHumidity()
{
    return HTS.readHumidity();
}

float Sensors::getPressure()
{
    //KILOPASCAL
    //MILLIBAR
    //PSI
    return BARO.readPressure(KILOPASCAL);
}

Coordinates Sensors::getAccelerationDirection()
{
    if (IMU.accelerationAvailable())
    {
        float x, y, z;
        IMU.readAcceleration(x, y, z);
        acceleration.X = x;
        acceleration.Y = y;
        acceleration.Z = z;
    }

    return acceleration;
}

Coordinates Sensors::getGyroscopeDirection()
{
    if (IMU.gyroscopeAvailable())
    {
        float x, y, z;
        IMU.readGyroscope(x, y, z);
        gyroscope.X = x;
        gyroscope.Y = y;
        gyroscope.Z = z;
    }

    return gyroscope;
}

Coordinates Sensors::getMagneticField()
{
    if (IMU.magneticFieldAvailable())
    {
        float x, y, z;
        IMU.readMagneticField(x, y, z);
        magneticField.X = x;
        magneticField.Y = y;
        magneticField.Z = z;
    }

    return magneticField;
}

Coordinates Sensors::getCalibratedMagneticField()
{
    Coordinates coords = getMagneticField();

    float x = CalculateNormalizedValue(coords.X, magneticFieldMinValues.X, magneticFieldMaxValues.X);
    float y = CalculateNormalizedValue(coords.Y, magneticFieldMinValues.Y, magneticFieldMaxValues.Y);
    float z = CalculateNormalizedValue(coords.Z, magneticFieldMinValues.Z, magneticFieldMaxValues.Z);

    magneticField.X = x;
    magneticField.Y = y;
    magneticField.Z = z;

    return magneticField;
}

void Sensors::CalibrateMagnetoMeter()
{
    delay(1000);
}

bool Sensors::initializeAll()
{
    bool initializedTemp = initializeTempAndHumiditySensor();
    bool initializedIMU = initializeIMUSensor();
    bool initializedBaro = initializeBarometricSensor();
    return initializedTemp && initializedIMU && initializedBaro;
}

bool Sensors::initializeIMUSensor()
{
    if (IMU.begin())
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Sensors::initializeBarometricSensor()
{
    if (BARO.begin())
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Sensors::initializeTempAndHumiditySensor()
{
    if (HTS.begin())
    {
        return true;
    }
    else
    {
        return false;
    }
}

float Sensors::CalculateNormalizedValue(float value, float min, float max)
{
    return 2 * ((value - min) / (max - min)) - 1;
}