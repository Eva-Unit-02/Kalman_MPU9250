#include <Arduino.h>
#include <Wire.h>
#include "MPU9250_IMU.h"

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

// Gyroscope full scale definitions
#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

// Accelerometer full scale definitions 
#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

MPU9250_IMU::MPU9250_IMU() {
  MPU9250_IMU(1000);
}

MPU9250_IMU::MPU9250_IMU(int numSamples) {
  _numSamples = numSamples;
}

void MPU9250_IMU::begin() {
  Wire.begin();

  // Configure accelerometer
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);
  // Configure gyroscope
  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
  // Configure magnetometer
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);  // Enable I2C bypass
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);      // Set magnetometer to single measurement mode

  Calibrate();
}

void MPU9250_IMU::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

void MPU9250_IMU::I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void MPU9250_IMU::GetReading(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& mx, float& my, float& mz) {
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

  int16_t ax_i = (Buf[0] << 8 | Buf[1]);
  int16_t ay_i = (Buf[2] << 8 | Buf[3]);
  int16_t az_i = (Buf[4] << 8 | Buf[5]);

  int16_t gx_i = (Buf[8] << 8 | Buf[9]);
  int16_t gy_i = (Buf[10] << 8 | Buf[11]);
  int16_t gz_i = (Buf[12] << 8 | Buf[13]);

  uint8_t ST1;
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);
  do {
    I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
  } while (!(ST1 & 0x01));

  uint8_t Mag[7];
  I2Cread(MAG_ADDRESS, 0x03, 7, Mag);

  int16_t mx_i = -(Mag[3] << 8 | Mag[2]);
  int16_t my_i = -(Mag[1] << 8 | Mag[0]);
  int16_t mz_i = -(Mag[5] << 8 | Mag[4]);

  ax = (ax_i / 2048.0) - accelOffsetX;
  ay = (ay_i / 2048.0) - accelOffsetY;
  az = (az_i / 2048.0) - accelOffsetZ;

  gx = (gx_i / 16.4) - gyroOffsetX;
  gy = (gy_i / 16.4) - gyroOffsetY;
  gz = (gz_i / 16.4) - gyroOffsetZ;

  mx = (mx_i / 0.6);
  my = (my_i / 0.6);
  mz = (mz_i / 0.6);
}

void MPU9250_IMU::Calibrate() {
  Serial.println("Calibration Started");
  float axSum = 0, aySum = 0, azSum = 0;
  float gxSum = 0, gySum = 0, gzSum = 0;

  for (int i = 0; i < _numSamples; i++) {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    GetReading(ax, ay, az, gx, gy, gz, mx, my, mz);
    
    axSum += ax;
    aySum += ay;
    azSum += az;
    gxSum += gx;
    gySum += gy;
    gzSum += gz;
    
    delay(5);
  }

  accelOffsetX = axSum / _numSamples;
  accelOffsetY = aySum / _numSamples;
  accelOffsetZ = (azSum / _numSamples) - 1.0;
  
  gyroOffsetX = gxSum / _numSamples;
  gyroOffsetY = gySum / _numSamples;
  gyroOffsetZ = gzSum / _numSamples;

  Serial.println("Calibration Ended");
}