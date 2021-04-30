#include "QuadCopterControl.h"
#include "Sensors.h"
#include "MultiMotorControl.h"
#include "QuadCopterTypes.h"

#define HEIGHT 10
#define LENGTH 10
#define A (HEIGHT * DeltaT / (2 * IXX)) // Constant a c t ing on pi t ch
#define B (LENGTH * DeltaT / (2 * IYY)) // Constant a c t ing on r o l l

QuadCopterControl::QuadCopterControl(uint8_t *motorPins, uint8_t height, uint8_t length)
{
    functions = new MathFunctions();
    motors = new MultiMotorControl(4, motorPins, false);
    sensors = new Sensors();

    imuControlValues.Roll = 0;
    imuControlValues.Yaw = 0;
    imuControlValues.Pitch = 0;

    remoteControlValues.Roll = 0;
    remoteControlValues.Yaw = 0;
    remoteControlValues.Pitch = 0;
}

void QuadCopterControl::Initilize()
{
    sensors->initializeAll();
}

void QuadCopterControl::Yaw(int8_t targetDegrees)
{
    remoteControlValues.Yaw = targetDegrees;
}

void QuadCopterControl::Roll(int8_t targetDegrees)
{
    remoteControlValues.Roll = targetDegrees;
}

void QuadCopterControl::Pitch(int8_t targetDegrees)
{
    remoteControlValues.Pitch = targetDegrees;
}

void QuadCopterControl::Thrust(int8_t power)
{
    remoteControlValues.Throttle = power;
}

void QuadCopterControl::GetControlErrors()
{
    float controlRoll = functions.CalculatePID(imuControlValues.Roll, remoteControlValues.Roll, rollPid);
    float controlPitch = functions.CalculatePID(imuControlValues.Pitch, remoteControlValues.Pitch, pitchPid);
    float controlYaw = functions.CalculatePID(imuControlValues.Yaw, remoteControlValues.Yaw, yawPid);

    ApplyRoll(controlRoll);
    ApplyPitch(controlPitch);
    ApplyYaw(controlYaw);
}

void QuadCopterControl::ApplyPitch(float controlPitch)
{
    uint8_t throttle = 1 / controlPitch

                       * motors.getMotor(0).Up();
    *motors.getMotor(1).
         *motors.getMotor(2)
             .
                 *motors.getMotor(3)
             .
}

void QuadCopterControl::ApplyRoll(float controlRoll)
{
}

void QuadCopterControl::ApplyYaw(float controlYaw)
{
}
