#ifndef CONST_CONFIG
#define CONST_CONFIG

#include "Arduino.h"

namespace ConstConfig
{
    const float ACC_COEFICIENT = 0.01;
    const float GYRO_COEFICIENT = 0.99;
    const float DELTA_T = 0.002;
    const float INTEGRAL_THRESHOLD = 200;
    const float FILTERS_COEFICIENT = 0.1;
    const float Pi = 3.1415926535897932384626433832795;
    const float RADIANS = 180 / Pi;
    const float GRAVITY = 9.8;
    const int MAGNETOMETER_CALIB_TIME_MS = 8000;

    const float QUAD_MASS = 2000;
    const uint8_t MOTORS_COUNT = 4;
};

#endif
//#define PI 3.1415926535897932384626433832795