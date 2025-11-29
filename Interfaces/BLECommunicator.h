#ifndef BLE_COMMUNICATOR
#define BLE_COMMUNICATOR

#include <ArduinoBLE.h>
#include "../Models/Coordinates.h"
#include "../Interfaces/Sensors.h"
#include "../Interfaces/MultiMotorControl.h"
#include "../Constants/BLEConstants.h"


class BLECommunicator
{
public:
    BLECommunicator(Sensors *sensors, MultiMotorControl *motors);
    ~BLECommunicator();
    void setTemperature(int8_t temperature);
    void setHumidity(int8_t humidity);
    void setPressure(float pressure);
    void setAccerelationDirection(Coordinates coords);
    void setGyroscopeDirection(Coordinates coords);
    void setMagnetometerDirection(Coordinates coords);
    bool tryReadMotorSpeeds(uint8_t &motorPercent);
    void listenForConnections();

private:
    int _temperature = 0;
    float _pressure = 0;
    int _humidity = 0;
    Coordinates _accCoords;
    Coordinates _magCoords;
    Coordinates _gyroCoords;

    Sensors *_sensors;
    MultiMotorControl *_motors;

    // Use concrete members instead of pointers to avoid dangling references
    BLEService sensorsService{BLEConstants::SENSORS_SERVICE_UUID};
    BLEService commandsService{BLEConstants::COMMANDS_SERVICE_UUID};

    BLECharacteristic temperatureCharacteristic{BLEConstants::TEMPERATURE_CHAR_UUID, BLERead | BLENotify, sizeof(int8_t)};
    BLECharacteristic humidityCharacteristic{BLEConstants::HUMIDITY_CHAR_UUID, BLERead | BLENotify, sizeof(int8_t)};
    BLECharacteristic pressureCharacteristic{BLEConstants::PRESSUERE_CHAR_UUID, BLERead | BLENotify, sizeof(float)};
    BLECharacteristic accCoordinatesCharacteristic{BLEConstants::ACC_COORDS_UUID, BLERead | BLENotify, sizeof(Coordinates)};
    BLECharacteristic magCoordinatesCharacteristic{BLEConstants::MAG_COORDS_UUID, BLERead | BLENotify, sizeof(Coordinates)};
    BLECharacteristic gyroCoordinatesCharacteristic{BLEConstants::GYRO_COORDS_UUID, BLERead | BLENotify, sizeof(Coordinates)};
    BLECharacteristic motorSpeedCharacteristic{BLEConstants::MOTORS_SPEED_UUID, BLERead | BLEWrite, sizeof(uint8_t), true};
    void rampAllMotors(uint8_t throttle);
    //void motorsThrottled(BLEDevice central, BLECharacteristic characteristic);
};

// template BLETypedCharacteristic<float>;
// template BLETypedCharacteristic<int>;
// template BLETypedCharacteristic<Coordinates>;

#endif