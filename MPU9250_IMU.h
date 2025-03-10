#ifndef MPU9250_IMU_H
#define MPU9250_IMU_H

#include <Arduino.h>
#include <Wire.h>

class MPU9250_IMU {
  public:
    MPU9250_IMU();
    MPU9250_IMU(int numSamples);
    void begin();
    void GetReading(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& mx, float& my, float& mz);
    

  private:
    int _numSamples;
    void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
    void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
    void Calibrate();
    
    float accelOffsetX, accelOffsetY, accelOffsetZ;
    float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
};

#endif