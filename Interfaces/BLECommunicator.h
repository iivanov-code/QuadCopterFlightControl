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

    BLEService *sensorsService;
    BLEService *commandsService;

    BLETypedCharacteristic<int8_t> *_temperatureCharacteristic;
    BLETypedCharacteristic<int8_t> *_humidityCharacteristic;
    BLETypedCharacteristic<float> *_pressureCharacteristic;
    BLETypedCharacteristic<Coordinates> *_accCoordinatesCharacteristic;
    BLETypedCharacteristic<Coordinates> *_magCoordinatesCharacteristic;
    BLETypedCharacteristic<Coordinates> *_gyroCoordinatesCharacteristic;
    BLECharacteristic *_motorSpeedCharacteristic;
    void rampAllMotors(uint8_t throttle);
    //void motorsThrottled(BLEDevice central, BLECharacteristic characteristic);
};

// template BLETypedCharacteristic<float>;
// template BLETypedCharacteristic<int>;
// template BLETypedCharacteristic<Coordinates>;

#endif