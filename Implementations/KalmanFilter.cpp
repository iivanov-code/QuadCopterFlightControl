#include "../Interfaces/KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    // Initial state
    angle = 0.0;
    bias = 0.0;

    // Process noise covariance matrix
    Q_angle = 0.001;
    Q_bias = 0.003;

    // Measurement noise covariance
    R_measure = 0.03;

    // Error covariance matrix
    P[0][0] = 0.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 0.0;
}

// Update the filter with new measurements
// newAngle: angle from accelerometer (degrees)
// newRate: angular rate from gyroscope (degrees/s)
// dt: time step (seconds)
float KalmanFilter::Update(float newAngle, float newRate, float dt)
{
    // Prediction step
    // Predict the state
    rate = newRate - bias;
    angle += dt * rate;

    // Update the error covariance
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update step
    // Calculate Kalman gain
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Update estimate with measurement
    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    // Update error covariance
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

// Set the angle directly (for initialization)
void KalmanFilter::SetAngle(float newAngle)
{
    angle = newAngle;
}

// Get current angle estimate
float KalmanFilter::GetAngle()
{
    return angle;
}

// Get current rate estimate
float KalmanFilter::GetRate()
{
    return rate;
}

// Get gyro bias estimate
float KalmanFilter::GetBias()
{
    return bias;
}

// Adjust filter parameters for tuning
void KalmanFilter::SetQangle(float Q)
{
    Q_angle = Q;
}

void KalmanFilter::SetQbias(float Q)
{
    Q_bias = Q;
}

void KalmanFilter::SetRmeasure(float R)
{
    R_measure = R;
}