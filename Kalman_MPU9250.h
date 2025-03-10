#ifndef KALMAN_MPU9250_H
#define KALMAN_MPU9250_H

#include <Arduino.h>
#include "MPU9250_IMU.h"
#include "Kalman.h"

// Kalman filter for roll, pitch, and yaw of IMU
class Kalman_MPU9250 {
    public:
        Kalman_MPU9250();
        Kalman_MPU9250(int numSamples);
        Kalman_MPU9250(int numSamples, 
          float Q_angleRoll, float Q_biasRoll, float R_measureRoll,
          float Q_anglePitch, float Q_biasPitch, float R_measurePitch,
          float Q_angleYaw, float Q_biasYaw, float R_measureYaw);
        void begin();
        void update();
        float getRoll();
        float getPitch();
        float getYaw();
        float getRawRoll();
        float getRawPitch();
        float getRawYaw();

    private:
      Kalman kalmanRoll;
      Kalman kalmanPitch;
      Kalman kalmanYaw;

      MPU9250_IMU imu;
      
      unsigned long timer;
      float rawRoll, rawPitch, rawYaw;
      float fusedRoll, fusedPitch, fusedYaw;
};

#endif