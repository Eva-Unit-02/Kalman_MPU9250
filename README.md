# Kalman Filter for MPU9250 IMU

This Arduino library provides a Kalman filter implementation for the MPU9250 IMU sensor, offering accurate orientation estimation (roll, pitch, and yaw) by fusing accelerometer and gyroscope data.

## Features

- Kalman filter implementation for 3-axis orientation estimation
- Support for MPU9250 IMU sensor
- Configurable filter parameters for optimal performance
- Raw and filtered angle outputs
- Automatic calibration support
- Easy-to-use interface

## Installation

1. Download this repository as a ZIP file
2. In the Arduino IDE, go to Sketch > Include Library > Add .ZIP Library
3. Select the downloaded ZIP file

## Hardware Requirements

- Arduino-compatible board
- MPU9250 IMU sensor
- I2C connections (SDA and SCL pins)

## Usage

### Basic Usage

```cpp
#include <Kalman_MPU9250.h>

// Create IMU object with default settings
Kalman_MPU9250 IMU();

// Or specify number of samples for calibration
Kalman_MPU9250 IMU(500);

// Or specify both samples and Kalman filter parameters
Kalman_MPU9250 IMU(500, 0.001, 0.003, 0.03, 0.001, 0.003, 0.03, 0.001, 0.003, 0.03);

void setup() {
  Serial.begin(115200);
  IMU.begin();
}

void loop() {
  IMU.update();
  
  // Get filtered angles
  float roll = IMU.getRoll();
  float pitch = IMU.getPitch();
  float yaw = IMU.getYaw();
  
  // Get raw angles
  float rawRoll = IMU.getRawRoll();
  float rawPitch = IMU.getRawPitch();
  float rawYaw = IMU.getRawYaw();
}
```

### Available Methods

- `begin()`: Initialize the IMU and Kalman filter
- `update()`: Update the sensor readings and filter calculations
- `getRoll()`: Get filtered roll angle
- `getPitch()`: Get filtered pitch angle
- `getYaw()`: Get filtered yaw angle
- `getRawRoll()`: Get raw roll angle
- `getRawPitch()`: Get raw pitch angle
- `getRawYaw()`: Get raw yaw angle

## Kalman Filter Parameters

The Kalman filter parameters can be customized for each axis:
- Q_angle: Process noise covariance for the angle
- Q_bias: Process noise covariance for the bias
- R_measure: Measurement noise covariance

Default values are provided, but you can adjust them based on your specific requirements and sensor characteristics.

## Examples

Check the `examples` folder for complete usage examples.

## License

This project is open source and available under the MIT License.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
 
