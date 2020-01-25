#include "math.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define left_motor_A 6
#define left_motor_B 9
#define right_motor_A 10
#define right_motor_B 11

MPU6050 IMU;

// PWM Values for Each Motor [0, 255]
int16_t left_motor_speed = 0;
int16_t right_motor_speed = 0;

// accelerometer readings
int16_t x, y, z;

// angles
float current_angle = 0;
float last_angle = 0;
float set_angle = 0;

// errors
float this_error = 0;
float last_error = 0;
float error_sum = 0;

// PID Constants
unsigned int kp = 40;
unsigned int ki = 40;
float kd = 0.05;

// timing
unsigned long last_update = 0;
unsigned long update_interval = 5;
float update_interval_s = 0.005;

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
  // execute every 5ms
  if ((millis() - last_update) >= 5) {


    // calculate the current angle
    x = IMU.getAccelerationX();
    y = IMU.getAccelerationY();
    current_angle = atan2(x, y) * RAD_TO_DEG;
    this_error = current_angle - set_angle;
    error_sum += this_error;

    left_motor_speed = kp * (this_error) + ki * (error_sum) * update_interval_s - kd * (current_angle - last_angle) / update_interval_s;
    right_motor_speed = left_motor_speed;
    last_angle = current_angle;
  }
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
