#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 IMU;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {

  Serial.begin(9600);

  Serial.println("Initializing...");
  IMU.initialize();

  Serial.println("Testing device connections...");
  Serial.println(IMU.testConnection() ? "Connection successful" : "Connection failed");
}

void loop() {
  IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.println(ax);
  delay(100);
}
