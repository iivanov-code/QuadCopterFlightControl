#ifndef KALMAN_FILTER
#define KALMAN_FILTER

class KalmanFilter
{
public:
    KalmanFilter();

    // Update the filter with new measurements
    // newAngle: angle from accelerometer (degrees)
    // newRate: angular rate from gyroscope (degrees/s)
    // dt: time step (seconds)
    float Update(float newAngle, float newRate, float dt);
    
    // Set the angle directly (for initialization)
    void SetAngle(float newAngle);

    
    // Get current angle estimate
    float GetAngle();
    
    // Get current rate estimate
    float GetRate();
    
    // Get gyro bias estimate
    float GetBias();
    
    // Adjust filter parameters for tuning
    void SetQangle(float Q);
    
    void SetQbias(float Q);
    
    void SetRmeasure(float R);

private:
    // State variables
    float angle;      // Estimated angle
    float bias;       // Estimated gyro bias
    float rate;       // Unbiased rate
    
    // Covariance matrix
    float P[2][2];
    
    // Noise parameters
    float Q_angle;    // Process noise variance for angle
    float Q_bias;     // Process noise variance for bias
    float R_measure;  // Measurement noise variance
};

#endif
