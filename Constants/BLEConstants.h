#ifndef BLE_CONSTANTS
#define BLE_CONSTANTS

namespace BLEConstants
{
    const char *BLUETOOTH_NAME = "ArduinoNano";
    const int GENERIC_AIRCRAFT = 0x0980;

    const char *SENSORS_SERVICE_UUID = "7eeb4fd7-203e-4220-b375-a9889882d425";
    const char *TEMPERATURE_CHAR_UUID = "0x2A6E"; //"0c56f4f1-bebe-431a-8cf5-d3f113e88487";
    const char *HUMIDITY_CHAR_UUID = "0x2A6F";    //"61ff185d-6402-4cf8-ad45-120ab9fdd89d";
    const char *PRESSUERE_CHAR_UUID = "0x2A6D";   //"e6e8ecaf-bd9d-4844-80b4-f80dd28bb063";
    const char *ACC_COORDS_UUID = "91f46c84-33a7-4d49-9104-05c431c83c68";
    const char *GYRO_COORDS_UUID = "0e00d5dc-68f2-4cc9-a72a-fb30c2c73d4b";
    const char *MAG_COORDS_UUID = "348c13fa-3c67-47b1-9c39-91ebdcfeef4c";

    const char *CELSIUS_UUID = "0x272F";
    const char *PASCAL_UUID = "0x2724";
    const char *PERCENTAGE_UUID = "0x27AD";

    const char *COMMANDS_SERVICE_UUID = "09465f41-18a5-4833-b761-5d3bf31f92a6";
    const char *MOTORS_SPEED_UUID = "8f5f1171-dec5-45ef-8247-238b08fae64e";
};

#endif