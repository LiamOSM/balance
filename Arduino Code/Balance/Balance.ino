#include "math.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 IMU;

int16_t x, y, z;
float current_angle;

void setup() {
  Serial.begin(9600);
  IMU.initialize();
}

void loop() {
  if (IMU.testConnection()) {
    x = IMU.getAccelerationX();
    y = IMU.getAccelerationY();
    z = IMU.getAccelerationZ();
    current_angle = atan2(x, y) * RAD_TO_DEG;
    Serial.print("Angle: ");
    Serial.print(current_angle);
    Serial.println("º");
  }
  delay(1000);
}
