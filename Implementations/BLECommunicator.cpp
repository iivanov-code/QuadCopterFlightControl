#include "../Interfaces/BLECommunicator.h"

BLECommunicator::BLECommunicator(Sensors *sensors, MultiMotorControl *motors)
{
    _sensors = sensors;
    _motors = motors;

    if (BLE.begin())
    {
        BLE.setLocalName(BLEConstants::BLUETOOTH_NAME);
        BLE.setDeviceName(BLEConstants::BLUETOOTH_NAME);
        BLE.setAppearance(BLEConstants::GENERIC_AIRCRAFT);
        BLE.setAdvertisedService(sensorsService);

        sensorsService.addCharacteristic(temperatureCharacteristic);
        sensorsService.addCharacteristic(humidityCharacteristic);
        sensorsService.addCharacteristic(pressureCharacteristic);
        sensorsService.addCharacteristic(accCoordinatesCharacteristic);
        sensorsService.addCharacteristic(magCoordinatesCharacteristic);
        sensorsService.addCharacteristic(gyroCoordinatesCharacteristic);

        BLE.addService(sensorsService);

        commandsService.addCharacteristic(motorSpeedCharacteristic);
        BLE.addService(commandsService);

        // Initialize values
        temperatureCharacteristic.writeValue(_temperature);
        humidityCharacteristic.writeValue(_humidity);
        pressureCharacteristic.writeValue(_pressure);
        accCoordinatesCharacteristic.writeValue((uint8_t *)&_accCoords, sizeof(Coordinates));
        magCoordinatesCharacteristic.writeValue((uint8_t *)&_magCoords, sizeof(Coordinates));
        gyroCoordinatesCharacteristic.writeValue((uint8_t *)&_gyroCoords, sizeof(Coordinates));
        uint8_t motor = 0;
        motorSpeedCharacteristic.writeValue(motor);

        BLE.advertise();
    }
}

BLECommunicator::~BLECommunicator()
{
    BLE.stopAdvertise();
    BLE.end();
}

// void BLECommunicator::motorsThrottled(BLEDevice central, BLECharacteristic characteristic)
// {
//     uint8_t throttle = characteristic.value<uint8_t>();
//     rampAllMotors(throttle);
// }

void BLECommunicator::listenForConnections()
{
    // listen for BLE peripherals to connect:
    BLEDevice central = BLE.central();

    // if a central is connected to peripheral:
    if (central)
    {
        // while the central is still connected to peripheral:
        while (central.connected())
        {
            // if the remote device wrote to the characteristic,
            // use the value to control the LED:
            if (motorSpeedCharacteristic.written())
            {
                uint8_t motorPercentage = 0;
                motorSpeedCharacteristic.readValue(motorPercentage);
            }

            if (_temperature != _sensors->GetTemperature())
            {
                _temperature = _sensors->GetTemperature();
                temperatureCharacteristic.writeValue(_temperature);
            }

            if (_pressure != _sensors->GetPressure())
            {
                _pressure = _sensors->GetPressure();
                pressureCharacteristic.writeValue(_pressure);
            }

            if (_humidity != _sensors->GetHumidity())
            {
                _humidity = _sensors->GetHumidity();
                humidityCharacteristic.writeValue(_humidity);
            }

            delay(500);
        }
    }
}

void BLECommunicator::rampAllMotors(uint8_t throttle)
{
    _motors->getMotor(0)->SetThrottlePercent(throttle);
    _motors->getMotor(1)->SetThrottlePercent(throttle);
    _motors->getMotor(2)->SetThrottlePercent(throttle);
    _motors->getMotor(3)->SetThrottlePercent(throttle);
}

void BLECommunicator::setTemperature(int8_t temperature)
{
    if (temperature != _temperature)
    {
        this->temperatureCharacteristic.writeValue(temperature);
    }
}

void BLECommunicator::setPressure(float pressure)
{
    if (pressure != _pressure)
    {
        this->pressureCharacteristic.writeValue(pressure);
    }
}

void BLECommunicator::setHumidity(int8_t humidity)
{
    if (humidity != _humidity)
    {
        this->humidityCharacteristic.writeValue(humidity);
    }
}

void BLECommunicator::setAccerelationDirection(Coordinates coords)
{
    accCoordinatesCharacteristic.writeValue((uint8_t *)&coords, sizeof(Coordinates));
}

void BLECommunicator::setGyroscopeDirection(Coordinates coords)
{
    gyroCoordinatesCharacteristic.writeValue((uint8_t *)&coords, sizeof(Coordinates));
}

void BLECommunicator::setMagnetometerDirection(Coordinates coords)
{
    magCoordinatesCharacteristic.writeValue((uint8_t *)&coords, sizeof(Coordinates));
}

bool BLECommunicator::tryReadMotorSpeeds(uint8_t &motorPercent)
{
    if (motorSpeedCharacteristic.valueUpdated())
    {
        motorSpeedCharacteristic.readValue(motorPercent);
        return true;
    }

    return false;
}