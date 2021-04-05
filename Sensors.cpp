#include "Sensors.h"

Sensors::Sensors()
{
    acceleration.X = 0;
    acceleration.Y = 0;
    acceleration.Z = 0;
    gyroscope.X = 0;
    gyroscope.Y = 0;
    gyroscope.Z = 0;
    magneticField.X = 0;
    magneticField.Y = 0;
    magneticField.Z = 0;
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

float Sensors::getPressure()
{
    //KILOPASCAL
    //MILLIBAR
    //PSI
    return BARO.readPressure(KILOPASCAL);
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