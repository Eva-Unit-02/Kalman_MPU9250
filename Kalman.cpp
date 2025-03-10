#include <Arduino.h>
#include "Kalman.h"

Kalman::Kalman() {
    _Q_angle = 0.001;
  
    _Q_bias = 0.003;

    _R_measure = 0.03;  // originally 0.03

    angle = 0.0; 
    bias  = 0.0;

    // Initialize covariance matrix to 0
    P[0][0] = 0;  P[0][1] = 0;
    P[1][0] = 0;  P[1][1] = 0;
}

Kalman::Kalman(float Q_angle, float Q_bias, float R_measure) {
    _Q_angle = Q_angle; // This is the process noise variance for the angle estimate. 
    // It represents the uncertainty in the process model for the angle. 
    // In other words, it quantifies how much the angle estimate might deviate 
    // from the true value due to unmodeled dynamics or external disturbances.
  
    _Q_bias = Q_bias; // This is the process noise variance for the gyroscope bias estimate.
    // It represents the uncertainty in the process model for the bias.
    // It quantifies how much the bias estimate might change over time
    // due to factors like sensor drift or temperature variations.
  
    // Assumed noise variance for real sensor data. 
    // Smaller value means trusting sensor data more compared to model prediction
    _R_measure = R_measure;  // originally 0.03

    angle = 0.0; 
    bias  = 0.0;

    // Initialize covariance matrix to 0
    P[0][0] = 0;  P[0][1] = 0;
    P[1][0] = 0;  P[1][1] = 0;
}
  
double Kalman::getAngle(double newAngle, double newRate, double dt) {
    // Predict phase:
    rate = newRate - bias;
    angle += dt * rate;
    
    // Update the error covariance matrix
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + _Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += _Q_bias * dt;
    
    // Compute the Kalman gain
    double S = P[0][0] + _R_measure;    // Estimate error + measurement error
    double K0 = P[0][0] / S; // This is the Kalman gain for the angle estimate. 
    // It determines how much weight is given to the new measurement (from the accelerometer) when updating the angle estimate.
    
    double K1 = P[1][0] / S; // This is the Kalman gain for the bias estimate. 
    // It determines how much weight is given to the new measurement when updating the bias estimate.
    
    // Update phase:
    double y = newAngle - angle;         // Angle difference
    angle += K0 * y;
    bias  += K1 * y;
    
    // Update error covariance matrix
    double P00_temp = P[0][0];
    double P01_temp = P[0][1];
    
    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;
    
    return angle;
}