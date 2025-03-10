#include <Arduino.h>
#include "Kalman_MPU9250.h"

Kalman_MPU9250::Kalman_MPU9250() {
  imu = MPU9250_IMU();

  kalmanRoll = Kalman();  
  kalmanPitch = Kalman();
  kalmanYaw = Kalman();

  // Initialize timer with the current time in microseconds
  timer = micros();
}

Kalman_MPU9250::Kalman_MPU9250( int numSamples) {
  imu = MPU9250_IMU(numSamples);

  kalmanRoll = Kalman();
  kalmanPitch = Kalman();
  kalmanYaw = Kalman();

  // Initialize timer with the current time in microseconds
  timer = micros();
}

Kalman_MPU9250::Kalman_MPU9250(int numSamples, 
  float Q_angleRoll, float Q_biasRoll, float R_measureRoll,
  float Q_anglePitch, float Q_biasPitch, float R_measurePitch,
  float Q_angleYaw, float Q_biasYaw, float R_measureYaw) {

  imu = MPU9250_IMU(numSamples);

  // Initialize Kalman filters with provided parameters
  kalmanRoll = Kalman(Q_angleRoll, Q_biasRoll, R_measureRoll);
  kalmanPitch = Kalman(Q_anglePitch, Q_biasPitch, R_measurePitch);
  kalmanYaw = Kalman(Q_angleYaw, Q_biasYaw, R_measureYaw);

  // Initialize timer with the current time in microseconds
  timer = micros();
}


void Kalman_MPU9250::begin() {
  // Call the base class's begin function to initialize the MPU9250
  imu.begin();
}


void Kalman_MPU9250::update() {
  unsigned long now = micros();
  double dt = (now - timer) / 1000000.0;
  timer = now;
  
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  imu.GetReading(ax, ay, az, gx, gy, gz, mx, my, mz);
  
  rawRoll  = atan2(ay, az) * 180.0 / PI;
  rawPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  fusedRoll  = kalmanRoll.getAngle(rawRoll, gx, dt);
  fusedPitch = kalmanPitch.getAngle(rawPitch, gy, dt);


  // After computing fusedRoll and fusedPitch:
  double rollRad = fusedRoll * PI / 180.0;
  double pitchRad = fusedPitch * PI / 180.0;

  // Tilt compensation
  double mx_comp = mx * cos(pitchRad) + my * sin(rollRad) * sin(pitchRad) + mz * cos(rollRad) * sin(pitchRad);
  double my_comp = my * cos(rollRad) - mz * sin(rollRad);

  // Calculate yaw
  rawYaw = atan2(-my_comp, mx_comp) * 180.0 / PI;

  fusedYaw = kalmanYaw.getAngle(rawYaw, gz, dt);
}

float Kalman_MPU9250::getRoll() { return fusedRoll; }
float Kalman_MPU9250::getPitch() { return fusedPitch; }
float Kalman_MPU9250::getYaw() { return fusedYaw; }
float Kalman_MPU9250::getRawRoll() { return rawRoll; }
float Kalman_MPU9250::getRawPitch() { return rawPitch; }
float Kalman_MPU9250::getRawYaw() { return rawYaw; }