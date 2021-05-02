#include "Sensors.h"

int Sensors::GetTemperature()
{
    //CELSIUS
    //FAHRENHEIT
    return HTS.readTemperature(CELSIUS);
}

int Sensors::GetHumidity()
{
    return HTS.readHumidity();
}

float Sensors::GetPressure()
{
    //KILOPASCAL
    //MILLIBAR
    //PSI
    return BARO.readPressure(KILOPASCAL);
}

Coordinates Sensors::GetAccelerationDirection()
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

Coordinates Sensors::GetGyroscopeDirection()
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

Coordinates Sensors::GetMagneticField()
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

Coordinates Sensors::GetCalibratedMagneticField()
{
    Coordinates coords = GetMagneticField();

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
    Coordinates mag;
    int startTime = millis();
    while ((millis() - startTime) < ConstConfig::MAGNETOMETER_CALIB_TIME_MS) //Do calibration loop for 8 seconds
    {
        mag = GetMagneticField();
        SetMinMax(&magneticFieldMinValues.X, &magneticFieldMaxValues.X, mag.X);
        SetMinMax(&magneticFieldMinValues.Y, &magneticFieldMaxValues.Y, mag.Y);
        SetMinMax(&magneticFieldMinValues.Z, &magneticFieldMaxValues.Z, mag.Z);
        delay(100);
    }
}

bool Sensors::InitializeAll()
{
    bool initializedTemp = InitializeTempAndHumiditySensor();
    bool initializedIMU = InitializeIMUSensor();
    bool initializedBaro = InitializeBarometricSensor();
    return initializedTemp && initializedIMU && initializedBaro;
}

bool Sensors::InitializeIMUSensor()
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

bool Sensors::InitializeBarometricSensor()
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

bool Sensors::InitializeTempAndHumiditySensor()
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

void Sensors::SetMinMax(float *min, float *max, float value)
{
    if (value < *min)
    {
        *min = value;
    }

    if (value > *max)
    {
        *max = value;
    }
}

float Sensors::CalculateNormalizedValue(float value, float min, float max)
{
    return 2 * ((value - min) / (max - min)) - 1;
}