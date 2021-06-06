#include "BLECommunicator.h"
#include "BLEConstants.h"

BLECommunicator::BLECommunicator(Sensors *sensors, MultiMotorControl *motors)
{
    _sensors = sensors;
    _motors = motors;

    // begin initialization
    if (BLE.begin())
    {
        BLEService sensorsService(BLEConstants::SENSORS_SERVICE_UUID);
        BLEService commandsService(BLEConstants::COMMANDS_SERVICE_UUID);
        BLETypedCharacteristic<int8_t> temperatureCharacteristic(BLEConstants::TEMPERATURE_CHAR_UUID, BLERead | BLENotify);
        BLETypedCharacteristic<int8_t> humidityCharacteristic(BLEConstants::HUMIDITY_CHAR_UUID, BLERead | BLENotify);
        BLETypedCharacteristic<float> pressureCharacteristic(BLEConstants::PRESSUERE_CHAR_UUID, BLERead | BLENotify);
        BLETypedCharacteristic<Coordinates> accCoordinatesCharacteristic(BLEConstants::ACC_COORDS_UUID, BLERead | BLENotify);
        BLETypedCharacteristic<Coordinates> magCoordinatesCharacteristic(BLEConstants::MAG_COORDS_UUID, BLERead | BLENotify);
        BLETypedCharacteristic<Coordinates> gyroCoordinatesCharacteristic(BLEConstants::GYRO_COORDS_UUID, BLERead | BLENotify);
        BLECharacteristic motorSpeedCharacteristic(BLEConstants::MOTORS_SPEED_UUID, BLERead | BLEWrite, 1, true);

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

        // add service
        BLE.addService(sensorsService);

        commandsService.addCharacteristic(motorSpeedCharacteristic);
        //motorSpeedCharacteristic.setEventHandler(BLEWritten, BLECommunicator::motorsThrottled);

        BLE.addService(commandsService);

        // set the initial value for the characeristic:
        temperatureCharacteristic.writeValue(_temperature);
        humidityCharacteristic.writeValue(_humidity);
        pressureCharacteristic.writeValue(_pressure);
        accCoordinatesCharacteristic.writeValue(_accCoords);
        magCoordinatesCharacteristic.writeValue(_magCoords);
        gyroCoordinatesCharacteristic.writeValue(_gyroCoords);
        uint8_t motor = 0;
        motorSpeedCharacteristic.writeValue(motor);

        _temperatureCharacteristic = &temperatureCharacteristic;
        _humidityCharacteristic = &humidityCharacteristic;
        _pressureCharacteristic = &pressureCharacteristic;
        _accCoordinatesCharacteristic = &accCoordinatesCharacteristic;
        _magCoordinatesCharacteristic = &magCoordinatesCharacteristic;
        _gyroCoordinatesCharacteristic = &gyroCoordinatesCharacteristic;

        // start advertising
        BLE.advertise();
    }
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
            if (_motorSpeedCharacteristic->written())
            {
                uint8_t motorPercentage = 0;
                _motorSpeedCharacteristic->readValue(motorPercentage);
            }

            if (_temperature != _sensors->GetTemperature())
            {
                _temperature = _sensors->GetTemperature();
                _temperatureCharacteristic->writeValue(_temperature);
            }

            if (_pressure != _sensors->GetPressure())
            {
                _pressure = _sensors->GetPressure();
                _pressureCharacteristic->writeValue(_pressure);
            }

            if (_humidity != _sensors->GetHumidity())
            {
                _humidity = _sensors->GetHumidity();
                _humidityCharacteristic->writeValue(_humidity);
            }

            delay(500);
        }
    }
}

void BLECommunicator::rampAllMotors(uint8_t throttle)
{
    _motors->getMotor(0)->ChangeThrottle(throttle);
    _motors->getMotor(1)->ChangeThrottle(throttle);
    _motors->getMotor(2)->ChangeThrottle(throttle);
    _motors->getMotor(3)->ChangeThrottle(throttle);
}

void BLECommunicator::setTemperature(int8_t temperature)
{
    if (temperature != _temperature)
    {
        this->_temperatureCharacteristic->writeValue(temperature);
    }
}

void BLECommunicator::setPressure(float pressure)
{
    if (pressure != _pressure)
    {
        this->_humidityCharacteristic->writeValue(pressure);
    }
}

void BLECommunicator::setHumidity(int8_t humidity)
{
    if (humidity != _humidity)
    {
        this->_pressureCharacteristic->writeValue(humidity);
    }
}

void BLECommunicator::setAccerelationDirection(Coordinates coords)
{
    _accCoordinatesCharacteristic->writeValue(coords);
}

void BLECommunicator::setGyroscopeDirection(Coordinates coords)
{
    _gyroCoordinatesCharacteristic->writeValue(coords);
}

void BLECommunicator::setMagnetometerDirection(Coordinates coords)
{
    _magCoordinatesCharacteristic->writeValue(coords);
}

bool BLECommunicator::tryReadMotorSpeeds(uint8_t &motorPercent)
{
    if (_motorSpeedCharacteristic->valueUpdated())
    {
        _motorSpeedCharacteristic->readValue(motorPercent);
        return true;
    }

    return false;
}