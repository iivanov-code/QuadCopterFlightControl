#include "../../Interfaces/Controllers/KalmanQuadCopterControl.h"

KalmanQuadCopterControl::KalmanQuadCopterControl(uint8_t *motorPins)
    : BaseQuadCopterControl(motorPins)
{
    // Initialize Kalman filters for each axis
    rollKalman = new KalmanFilter();
    pitchKalman = new KalmanFilter();
    yawKalman = new KalmanFilter();

    // Initialize timing
    lastUpdateTime = micros();
    initialized = false;
}

KalmanQuadCopterControl::~KalmanQuadCopterControl()
{
    delete rollKalman;
    delete pitchKalman;
    delete yawKalman;
}

void KalmanQuadCopterControl::RunControlLoop()
{
    // Read sensor data
    Coordinates acc = sensors->GetAccelerationDirection();
    Coordinates gyro = sensors->GetGyroscopeDirection();
    Coordinates mag = sensors->GetCalibratedMagneticField();

    // Calculate time step
    unsigned long currentTime = micros();
    float dt = (currentTime - lastUpdateTime) / 1000000.0; // Convert to seconds
    lastUpdateTime = currentTime;

    // Clamp dt to reasonable values
    if (dt > 0.1)
        dt = 0.002; // Reset if too large (first iteration or overflow)
    if (dt < 0.0001)
        dt = 0.002; // Minimum dt

    // Calculate angles from accelerometer
    float accRoll = atan2(acc.Y, acc.Z) * 180.0 / PI;
    float accPitch = atan2(-acc.X, sqrt(acc.Y * acc.Y + acc.Z * acc.Z)) * 180.0 / PI;

    // Initialize Kalman filters on first run
    if (!initialized)
    {
        rollKalman->SetAngle(accRoll);
        pitchKalman->SetAngle(accPitch);
        yawKalman->SetAngle(0);
        initialized = true;
    }

    // Update Kalman filters with sensor fusion
    imuControlValues.Roll = rollKalman->Update(accRoll, gyro.X, dt);
    imuControlValues.Pitch = pitchKalman->Update(accPitch, gyro.Y, dt);

    // For yaw, use magnetometer with Kalman filter
    float magYaw = atan2(mag.Y, mag.X) * 180.0 / PI;
    imuControlValues.Yaw = yawKalman->Update(magYaw, gyro.Z, dt);

    // Apply base throttle
    ApplyThrottle();

    // Calculate and apply PID corrections
    ControlErrors errors = GetControlErrors();
    ApplyPitch(errors.ControlPitch);
    ApplyRoll(errors.ControlRoll);
    ApplyYaw(errors.ControlYaw);

    // Clamp motor values to valid range (0-255)
    for (int i = 0; i < ConstConfig::MOTORS_COUNT; i++)
    {
        if (motorThrottles[i] < 0)
            motorThrottles[i] = 0;
        if (motorThrottles[i] > 255)
            motorThrottles[i] = 255;
    }

    // Send to motors
    motors->getMotor(0)->SetThrottlePercent((uint8_t)motorThrottles[0]);
    motors->getMotor(1)->SetThrottlePercent((uint8_t)motorThrottles[1]);
    motors->getMotor(2)->SetThrottlePercent((uint8_t)motorThrottles[2]);
    motors->getMotor(3)->SetThrottlePercent((uint8_t)motorThrottles[3]);
}

ControlErrors KalmanQuadCopterControl::GetControlErrors()
{
    // Standard PID control using Kalman-filtered angles
    float controlRoll = MathFunctions::CalculatePID(
        imuControlValues.Roll,
        remoteControlValues.Roll,
        rollPid);

    float controlPitch = MathFunctions::CalculatePID(
        imuControlValues.Pitch,
        remoteControlValues.Pitch,
        pitchPid);

    float controlYaw = MathFunctions::CalculatePID(
        imuControlValues.Yaw,
        remoteControlValues.Yaw,
        yawPid);

    ControlErrors errors;
    errors.ControlRoll = controlRoll;
    errors.ControlPitch = controlPitch;
    errors.ControlYaw = controlYaw;

    return errors;
}