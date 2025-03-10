#include <Kalman_MPU9250.h>

// Kalman_MPU9250 IMU(); 
Kalman_MPU9250 IMU(500);  // Specify sample amount for calibration
// Kalman_MPU9250 IMU(500, 0.001, 0.003, 0.03, 0.001, 0.003, 0.03, 0.001, 0.003, 0.03); // Specify sample amount for calibration and Kalman constants for all 3 axis

void setup() {
  Serial.begin(115200);
  IMU.begin();
}

void loop() {
  IMU.update();

  Serial.print(IMU.getPitch());  // Fused pitch angle
  Serial.print(",");
  Serial.print(IMU.getRoll());   // Fused roll angle
  Serial.print(",");
  Serial.print(IMU.getYaw());    // Fused yaw angle

  Serial.print(",");

  Serial.print(IMU.getRawPitch());  // Raw pitch angle
  Serial.print(",");
  Serial.print(IMU.getRawRoll());   // Raw roll angle
  Serial.print(",");
  Serial.println(IMU.getRawYaw());  // Raw yaw angle

  delay(50);
}
