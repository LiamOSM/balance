#include "math.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define left_motor_A 6
#define left_motor_B 9
#define right_motor_A 10
#define right_motor_B 11

MPU6050 IMU;

// PWM Values for Each Motor [0, 255]
int16_t left_motor_speed;
int16_t right_motor_speed;

int16_t x, y, z;
float current_angle;

void setup() {

  // set data direction register
  pinMode(left_motor_A, OUTPUT);
  pinMode(left_motor_B, OUTPUT);
  pinMode(right_motor_A, OUTPUT);
  pinMode(right_motor_B, OUTPUT);

  Serial.begin(9600);

  IMU.initialize();

  if (!IMU.testConnection()) {
    Serial.println("Failed to connect");
    while (1) {}
  }
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

void updateMotors() {
  // set left motor speed
  if (left_motor_speed > 0) {
    digitalWrite(left_motor_A, LOW);
    analogWrite(left_motor_B, abs(left_motor_speed));
  }
  else {
    digitalWrite(left_motor_B, LOW);
    analogWrite(left_motor_A, abs(left_motor_speed));
  }

  // set right motor speed
  if (right_motor_speed > 0) {
    digitalWrite(right_motor_A, LOW);
    analogWrite(right_motor_B, abs(right_motor_speed));
  }
  else {
    digitalWrite(right_motor_B, LOW);
    analogWrite(right_motor_A, abs(right_motor_speed));
  }
}
