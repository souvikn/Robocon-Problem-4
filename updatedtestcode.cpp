#include <ESP32Servo.h>

// ==========================================
// 1. SERVO & IR LINE SENSOR SETUP
// ==========================================
Servo servo1;
const int servo_pin = 4;

const int irpin1 = 13;
const int irpin2 = 12;
const int irpin3 = 27;
const int irpin4 = 26;
const int irpin5 = 25;
const int irpin6 = 33;
const int irpin7 = 32;
const int irpin8 = 35;

int a[8];
int last_proportional = 0;
int integral = 0;
float steering_angle = 0;

const int max_angle = 30;
const int min_angle = -30;
const int center_angle = 90;

// Line Steering PID Gains
float Kp_line = 0.01; 
float Ki_line = 0.0;
float Kd_line = 0.0;

unsigned int position_val;
int derivative, proportional;

// ==========================================
// 2. CHASSIS PHYSICAL DIMENSIONS (in cm)
// ==========================================
const float WHEELBASE_L = 20.0;    // Distance between front & rear axles
const float TRACK_WIDTH_W = 15.0;  // Distance between left & right rear wheels
const float TICKS_PER_REV = 330.0; // Encoder ticks per 1 full wheel rotation

// ==========================================
// 3. BTS7960 & ENCODER PINS
// ==========================================
const int L_RPWM = 18; const int L_LPWM = 19; const int L_EN = 21; // Left Motor Driver
const int R_RPWM = 22; const int R_LPWM = 23; const int R_EN = 5;  // Right Motor Driver

const int ENC_LEFT_A = 14; const int ENC_LEFT_B = 15;
const int ENC_RIGHT_A = 16; const int ENC_RIGHT_B = 17;

// ==========================================
// 4. MOTOR SPEED PID SYSTEM VARIABLES
// ==========================================
float base_cruise_rpm = 400.0; // Base speed on straight paths

// Adjusted Motor PID Gains (Gently tuned to prevent instant 255 spikes)
float Kp_motor = 0.15, Ki_motor = 0.01, Kd_motor = 0.0;

// Tick Counters (updated automatically by interrupts)
volatile long left_ticks = 0;
volatile long right_ticks = 0;

// Motor PID Variables
float left_err_sum = 0, right_err_sum = 0;
float left_last_err = 0, right_last_err = 0;
float left_pwm = 0, right_pwm = 0;

unsigned long last_time = 0;

// Interrupt functions to count ticks
void IRAM_ATTR leftISR()  { (digitalRead(ENC_LEFT_B) == LOW) ? left_ticks++ : left_ticks--; }
void IRAM_ATTR rightISR() { (digitalRead(ENC_RIGHT_B) == LOW) ? right_ticks++ : right_ticks--; }

// Helper function to send PWM to BTS7960
void driveMotors(float pwmL, float pwmR) {
  // Clamped at 180 max PWM for safety headroom and traction control
  analogWrite(L_RPWM, constrain((int)pwmL, 0, 180)); analogWrite(L_LPWM, 0);
  analogWrite(R_RPWM, constrain((int)pwmR, 0, 180)); analogWrite(R_LPWM, 0);
}

// ==========================================
// 5. LINE SENSOR READ FUNCTION
// ==========================================
int readline() {
  a[0] = digitalRead(irpin1);
  a[1] = digitalRead(irpin2);
  a[2] = digitalRead(irpin3);
  a[3] = digitalRead(irpin4);
  a[4] = digitalRead(irpin5);
  a[5] = digitalRead(irpin6);
  a[6] = digitalRead(irpin7);
  a[7] = digitalRead(irpin8);
  
  int sum = (a[0] + a[1] + a[2] + a[3] + a[4] + a[5] + a[6] + a[7]);
  if (sum == 0) return 3500; // Default center if line is lost temporarily
  
  int v = (7000*a[0] + 6000*a[1] + 5000*a[2] + 4000*a[3] + 3000*a[4] + 2000*a[5] + 1000*a[6] + 0*a[7]) / sum;
  return v;
}

// ==========================================
// 6. SERVO WORKING FUNCTION
// ==========================================
float servoWorking(int p_difference) {
  int steering_offset = p_difference;
  steering_offset = constrain(steering_offset, min_angle, max_angle);

  int target_angle = center_angle + steering_offset;
  servo1.write(target_angle);

  return (float)steering_offset; // Returns actual physical steering angle (-30 to +30)
}

// ==========================================
// 7. CORE ACKERMAN SPEED & PID FUNCTION
// ==========================================
void updateWheelSpeeds(float steering_angle_deg) {
  unsigned long now = millis();
  if (now - last_time < 10) return; // Run every 10ms
  float dt = (now - last_time) / 1000.0; // Time in seconds (0.01s)
  last_time = now;

  // --- STAGE A: ACKERMAN KINEMATICS ---
  float rad = radians(steering_angle_deg);
  float factor = (TRACK_WIDTH_W * fabs(tan(rad))) / (2.0 * WHEELBASE_L);

  float target_left_rpm, target_right_rpm;
  if (steering_angle_deg >= 0) { // Turning Right
    target_left_rpm  = base_cruise_rpm * (1.0 + factor); // Outer wheel turns faster
    target_right_rpm = base_cruise_rpm * (1.0 - factor); // Inner wheel turns slower
  } else {                       // Turning Left
    target_left_rpm  = base_cruise_rpm * (1.0 - factor); // Inner wheel turns slower
    target_right_rpm = base_cruise_rpm * (1.0 + factor); // Outer wheel turns faster
  }

  // --- STAGE B: READ ENCODERS ---
  noInterrupts();
  long t_left = left_ticks;   left_ticks = 0;
  long t_right = right_ticks; right_ticks = 0;
  interrupts();

  // Convert ticks to real RPM
  float actual_left_rpm  = (t_left  / TICKS_PER_REV) * (60.0 / dt);
  float actual_right_rpm = (t_right / TICKS_PER_REV) * (60.0 / dt);

  // --- STAGE C: SPEED PID CONTROL ---
  // Left Wheel PID
  float err_left = target_left_rpm - actual_left_rpm;
  left_err_sum += err_left * dt;
  float deriv_left = (err_left - left_last_err) / dt;
  left_last_err = err_left;
  left_pwm += (err_left * Kp_motor) + (left_err_sum * Ki_motor) + (deriv_left * Kd_motor);

  // Right Wheel PID
  float err_right = target_right_rpm - actual_right_rpm;
  right_err_sum += err_right * dt;
  float deriv_right = (err_right - right_last_err) / dt;
  right_last_err = err_right;
  right_pwm += (err_right * Kp_motor) + (right_err_sum * Ki_motor) + (deriv_right * Kd_motor);

  // --- STAGE D: BTS DRIVER OUTPUT ---
  driveMotors(left_pwm, right_pwm);
}

// ==========================================
// 8. SETUP & LOOP
// ==========================================
void setup() {
  pinMode(irpin1, INPUT); pinMode(irpin2, INPUT); 
  pinMode(irpin3, INPUT); pinMode(irpin4, INPUT); 
  pinMode(irpin5, INPUT); pinMode(irpin6, INPUT); 
  pinMode(irpin7, INPUT); pinMode(irpin8, INPUT); 

  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT); pinMode(L_EN, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT); pinMode(R_EN, OUTPUT);

  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP); pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightISR, RISING);

  servo1.attach(servo_pin);
  Serial.begin(115200);
}

void loop() {
  // 1. Read Line Position
  position_val = readline();
  proportional = ((int)position_val - 3500);
  
  derivative = proportional - last_proportional;
  integral = integral + proportional;
  last_proportional = proportional;

  // 2. Line Steering PID Output
  int power_difference = proportional * Kp_line + integral * Ki_line + derivative * Kd_line;

  // 3. Move Front Servo and get steering angle
  steering_angle = servoWorking(power_difference);

  // 4. Update Rear Wheels via Ackerman Kinematics + Dual Motor PIDs
  updateWheelSpeeds(steering_angle);
}