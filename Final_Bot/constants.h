#include <Wire.h>
#include <SoftwareSerial.h>

// Define SoftwareSerial pins
SoftwareSerial bluetooth(A1, A0); // RX, TX

// Motor Pins
const int inputPin2 = A2; // Motor A, control pin 2
const int inputPin1 = A3; // Motor A, control pin 1
const int inputPin3 = 9;  // Motor B, control pin 3
const int inputPin4 = 4;  // Motor B, control pin 4
int EN1 = 6;             // Enable pin for Motor A, PWM control
int EN2 = 5;             // Enable pin for Motor B, PWM control

// Define encoder pins
#define encodPinAR 7  // Right encoder, channel A
#define encodPinBR 2  // Right encoder, channel B
#define encodPinAL 8  // Left encoder, channel A
#define encodPinBL 3  // Left encoder, channel B

// Variables for wheel angle and pulse count
int wheel_pulse_count_left = 0;
int wheel_pulse_count_right = 0;
float wheel_angle_left = 0.0, wheel_angle_old_left = 0.0, wheel_dot_left = 0.0;
float wheel_angle_right = 0.0, wheel_angle_old_right = 0.0, wheel_dot_right = 0.0;
const float WHEEL_RADIUS = 21.5; // Radius of the wheel in mm
const int ENCODER_PULSES_PER_REVOLUTION = 300; // Encoder resolution

// Tanays stuff
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTimeT, previousTimeT;
float t,lastt,dt;
float looptime_print;
float roll_offset = 0;
float tilt_error_print = 0.0;

float kp_tilt = 30;  // Proportional gain for tilt control - 15
float ki_tilt = 3;   // Integral gain for tilt control - 0
float kd_tilt = 1;   // Derivative gain for tilt control - 100
// kp 45, kd = 10, kp-pos 20

float kp_pos = 2;//-10;      // Proportional gain for position control
float ki_pos = 0.0;    // Integral gain for position control
float kd_pos = 0;//-20.0;    // Derivative in for position control

float alpha = 0.95;    // Weight for complementary filter
float desired_angle = 0.0; // Desired tilt angle for balance
float tilt_offset = 1.42;

float desired_pos = 0.0;   // Desired position of the bot
float integral_tilt = constrain(0.0,-20,20); // Integral term for tilt control
float integral_pos = 0.0;  // Integrl term for position control
float previousTiltError = 0.0; // Previous error for tilt PID
float previousPositionError = 0.0; // Previous error for position PID
unsigned long previousTime = 0; // Previous timestamp for PID calculation

float tilt_angle = 0;      // Current tilt angle
float current_position = 0.0; // Current position of the bot
float desired_position = 0.0; // Desired position of the bot

// Complementary Filter Variables
float filteredAngle = 0.0; // Filtered angle estimate
float gyroRateY = 0.0;     // Gyroscope rate for Y-axis
float beta = 1;            // Complementary filter weight (tunable)