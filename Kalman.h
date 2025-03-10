#ifndef KALMAN_H
#define KALMAN_H

#include <Arduino.h>

// Kalman filter for roll, pitch, and yaw of IMU
class Kalman {
    public:
        Kalman();
        Kalman(float Q_angle, float Q_bias, float R_measure);
        double getAngle(double newAngle, double newRate, double dt);

    private:
        double _Q_angle;    // Process noise variance for the accelerometer
        double _Q_bias;     // Process noise variance for the gyro bias
        double _R_measure;  // Measurement noise variance
        
        double angle;  // The angle calculated by the Kalman filter
        double bias;   // The gyro bias calculated by the Kalman filter
        double rate;   // Unbiased rate calculated from the rate and the calculated bias
        
        double P[2][2];  // Error covariance matrix
};

#endif