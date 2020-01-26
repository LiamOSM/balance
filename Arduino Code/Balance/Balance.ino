#include "math.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define left_motor_A 6
#define left_motor_B 9
#define right_motor_A 10
#define right_motor_B 11

// uncomment for serial debugging output
#define debug

MPU6050 IMU;

// PWM Values for Each Motor [0, 255]
int left_motor_speed = 0;
int right_motor_speed = 0;

// accelerometer readings
int16_t accx, accy, accz;
int16_t gyrox, gyroy, gyroz;

// angles
float acc_angle = 0;
float gyro_angle = 0;

float current_angle = 0;
float last_angle = 0;
float set_angle = 102.0;

int gyro_rate = 0;

// errors
float this_error = 0;
float last_error = 0;
float error_sum = 0;

// PID Constants
unsigned int kp = 10;
float ki = 0;
float kd = 0;

// timing
unsigned long last_update = 0;
unsigned long update_interval = 5;

void setup() {

  // set data direction register
  pinMode(left_motor_A, OUTPUT);
  pinMode(left_motor_B, OUTPUT);
  pinMode(right_motor_A, OUTPUT);
  pinMode(right_motor_B, OUTPUT);

  Serial.begin(9600);

  IMU.initialize();

  if (!IMU.testConnection()) {
#ifdef debug
    Serial.println("Failed to connect");
#endif
    while (1) {}
  }
#ifdef debug
  Serial.println("Setup successful, beginning control loop");
#endif
}

void loop() {
  //ramp_motors();
  // execute every 5ms
  if ((millis() - last_update) >= 500) {
    last_update = millis();

    // calculate the current angle
    // x = IMU.getAccelerationX();
    accy = IMU.getAccelerationY();
    accz = IMU.getAccelerationZ();
    acc_angle = atan2(accy, accz) * RAD_TO_DEG;
    current_angle = acc_angle;

#ifdef debug
    Serial.print("Current angle: ");
    Serial.print(current_angle);
    Serial.print("\tError: ");
    Serial.println(this_error);
#endif
    this_error = current_angle - set_angle;
    error_sum += this_error;
    left_motor_speed = kp * (this_error);
    right_motor_speed = left_motor_speed;
    last_angle = current_angle;
    update_motors();
  }
}

void update_motors() {
  // set left motor speed
  left_motor_speed = constrain(left_motor_speed, -255, 255);
  if (left_motor_speed < 0) {
    digitalWrite(left_motor_A, LOW);
    analogWrite(left_motor_B, abs(left_motor_speed));
  }
  else {
    digitalWrite(left_motor_B, LOW);
    analogWrite(left_motor_A, left_motor_speed);
  }

  // set right motor speed
  right_motor_speed = constrain(right_motor_speed, -255, 255);
  if (right_motor_speed < 0) {
    digitalWrite(right_motor_A, LOW);
    analogWrite(right_motor_B, abs(right_motor_speed));
  }
  else {
    digitalWrite(right_motor_B, LOW);
    analogWrite(right_motor_A, right_motor_speed);
  }

#ifdef debug
  Serial.print("Motor speed: ");
  Serial.println(left_motor_speed);
#endif
}

void ramp_motors() {
  while (1) {
    for (int i = -255; i < 255; i++) {
      left_motor_speed = i;
      right_motor_speed = i;
      update_motors();
      delay(25);
    }
    for (int i = 255; i > -255; i--) {
      left_motor_speed = i;
      right_motor_speed = i;
      update_motors();
      delay(25);
    }
  }
}
