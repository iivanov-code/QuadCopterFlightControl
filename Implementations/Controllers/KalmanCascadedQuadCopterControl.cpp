#include "../Interfaces/Controllers/KalmanCascadedQuadCopterControl.h"

KalmanCascadedQuadCopterControl::KalmanCascadedQuadCopterControl(uint8_t *motorPins)
    : BaseQuadCopterControl(motorPins)
{
    // Replace single PIDs with cascaded PIDs
    delete rollPid;
    delete pitchPid;
    delete yawPid;

    rollCascaded = new CascadedPIDModel();
    pitchCascaded = new CascadedPIDModel();
    yawCascaded = new CascadedPIDModel();

    // Initialize Kalman filters
    rollKalman = new KalmanFilter();
    pitchKalman = new KalmanFilter();
    yawKalman = new KalmanFilter();

    lastUpdateTime = micros();
    initialized = false;
}

KalmanCascadedQuadCopterControl::~KalmanCascadedQuadCopterControl()
{
    delete rollCascaded;
    delete pitchCascaded;
    delete yawCascaded;
    delete rollKalman;
    delete pitchKalman;
    delete yawKalman;
}

void KalmanCascadedQuadCopterControl::RunControlLoop() override
{
    // Read sensor data
    Coordinates acc = sensors->GetAccelerationDirection();
    Coordinates gyro = sensors->GetGyroscopeDirection();
    Coordinates mag = sensors->GetCalibratedMagneticField();

    // Calculate time step
    unsigned long currentTime = micros();
    float dt = (currentTime - lastUpdateTime) / 1000000.0;
    lastUpdateTime = currentTime;

    if (dt > 0.1)
        dt = 0.002;
    if (dt < 0.0001)
        dt = 0.002;

    // Calculate angles from accelerometer
    float accRoll = atan2(acc.Y, acc.Z) * 180.0 / PI;
    float accPitch = atan2(-acc.X, sqrt(acc.Y * acc.Y + acc.Z * acc.Z)) * 180.0 / PI;

    // Initialize on first run
    if (!initialized)
    {
        rollKalman->SetAngle(accRoll);
        pitchKalman->SetAngle(accPitch);
        yawKalman->SetAngle(0);
        initialized = true;
    }

    // Update Kalman filters
    imuControlValues.Roll = rollKalman->Update(accRoll, gyro.X, dt);
    imuControlValues.Pitch = pitchKalman->Update(accPitch, gyro.Y, dt);

    float magYaw = atan2(mag.Y, mag.X) * 180.0 / PI;
    imuControlValues.Yaw = yawKalman->Update(magYaw, gyro.Z, dt);

    // Store gyro rates for cascaded control
    gyroRates = gyro;

    // Apply base throttle
    ApplyThrottle();

    // Calculate and apply cascaded PID corrections
    ControlErrors errors = GetControlErrors();
    ApplyPitch(errors.ControlPitch);
    ApplyRoll(errors.ControlRoll);
    ApplyYaw(errors.ControlYaw);

    // Clamp motor values
    for (int i = 0; i < ConstConfig::MOTORS_COUNT; i++)
    {
        if (motorThrottles[i] < MIN_PERCENT)
            motorThrottles[i] = MIN_PERCENT;
        if (motorThrottles[i] > MAX_PERCENT)
            motorThrottles[i] = MAX_PERCENT;
    }

    // Send to motors
    motors->getMotor(0)->SetThrottlePercent((uint8_t)motorThrottles[0]);
    motors->getMotor(1)->SetThrottlePercent((uint8_t)motorThrottles[1]);
    motors->getMotor(2)->SetThrottlePercent((uint8_t)motorThrottles[2]);
    motors->getMotor(3)->SetThrottlePercent((uint8_t)motorThrottles[3]);
}

ControlErrors KalmanCascadedQuadCopterControl::GetControlErrors() override
{
    // Cascaded PID with Kalman-filtered angles

    // ROLL
    float rollDesiredRate = MathFunctions::CalculatePID(
        imuControlValues.Roll,
        remoteControlValues.Roll,
        rollCascaded->AnglePID);
    float controlRoll = MathFunctions::CalculatePID(
        gyroRates.X,
        rollDesiredRate,
        rollCascaded->RatePID);

    // PITCH
    float pitchDesiredRate = MathFunctions::CalculatePID(
        imuControlValues.Pitch,
        remoteControlValues.Pitch,
        pitchCascaded->AnglePID);
    float controlPitch = MathFunctions::CalculatePID(
        gyroRates.Y,
        pitchDesiredRate,
        pitchCascaded->RatePID);

    // YAW
    float yawDesiredRate = MathFunctions::CalculatePID(
        imuControlValues.Yaw,
        remoteControlValues.Yaw,
        yawCascaded->AnglePID);
    float controlYaw = MathFunctions::CalculatePID(
        gyroRates.Z,
        yawDesiredRate,
        yawCascaded->RatePID);

    ControlErrors errors;
    errors.ControlRoll = controlRoll;
    errors.ControlPitch = controlPitch;
    errors.ControlYaw = controlYaw;

    return errors;
}