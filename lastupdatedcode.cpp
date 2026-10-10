#include <ESP32Servo.h>

// ==========================================
// 1. SERVO & IR LINE SENSOR SETUP
// ==========================================
const int servo_pin = 15;

const int irpin1 = 25;
const int irpin2 = 26;
const int irpin3 = 27;
const int irpin4 = 14;
const int irpin5 = 12;
const int irpin6 = 13;
const int irpin7 = 2;
const int irpin8 = 4;

const int max_angle = 30;
const int min_angle = -30;
const int center_angle = 90;

// Line Steering PID Gains
float Kp_line = 1; 
float Ki_line = 0.0;
float Kd_line = 0.5;

// ==========================================
// 2. CHASSIS PHYSICAL DIMENSIONS (in cm)
// ==========================================
const float WHEELBASE_L = 20.0;    // Distance between front & rear axles
const float TRACK_WIDTH_W = 15.0;  // Distance between left & right rear wheels
const float TICKS_PER_REV = 30.0; // Encoder ticks per 1 full wheel rotation

// ==========================================
// 3. BTS7960 & ENCODER PINS
// ==========================================
const int L_RPWM = 3; const int L_LPWM = 21; const int L_EN = 1; const int R_EN1 = 22; // Left Motor Driver
const int R_RPWM = 17; const int R_LPWM = 16; const int R_EN = 18; const int L_EN1 = 5;   // Right Motor Driver

const int ENC_LEFT_A = 34; const int ENC_LEFT_B = 35;
const int ENC_RIGHT_A = 32; const int ENC_RIGHT_B = 33;

// ==========================================
// 4. MOTOR SPEED PID SYSTEM VARIABLES
// ==========================================
float base_cruise_rpm = 20.0; // Base speed on straight paths

// Adjusted Motor PID Gains (Gently tuned to prevent instant 255 spikes)
float Kp_motor = 1, Ki_motor = 0.01, Kd_motor = 0.5;


// ==========================================
// CLASS 1: LINE SENSOR (8 IR sensors)
// ==========================================
class LineSensor {
  private:
    int a[8];

  public:
    void begin() {
      pinMode(irpin1, INPUT); pinMode(irpin2, INPUT); 
      pinMode(irpin3, INPUT); pinMode(irpin4, INPUT); 
      pinMode(irpin5, INPUT); pinMode(irpin6, INPUT); 
      pinMode(irpin7, INPUT); pinMode(irpin8, INPUT); 
    }

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
};


// ==========================================
// CLASS 2: LINE STEERING PID
// ==========================================
class LinePID {
  private:
    int last_proportional;
    int integral;
    int derivative, proportional;

  public:
    LinePID() {
      last_proportional = 0;
      integral = 0;
    }

    int compute(unsigned int position_val) {
      proportional = ((int)position_val - 3500);
      
      derivative = proportional - last_proportional;
      integral = integral + proportional;
      last_proportional = proportional;

      int power_difference = proportional * Kp_line + integral * Ki_line + derivative * Kd_line;
      return power_difference;
    }
};


// ==========================================
// CLASS 3: STEERING SERVO
// ==========================================
class SteeringServo {
  private:
    Servo servo1;

  public:
    void begin() {
      servo1.attach(servo_pin);
    }

    float servoWorking(int p_difference) {
      int steering_offset = p_difference;
      steering_offset = constrain(steering_offset, min_angle, max_angle);

      int target_angle = center_angle + steering_offset;
      servo1.write(target_angle);

      return (float)steering_offset; // Returns actual physical steering angle (-30 to +30)
    }
};


// ==========================================
// CLASS 4: BTS7960 MOTOR DRIVER
// ==========================================
class MotorDriver {
  private:
    int rpwm_pin, lpwm_pin, en_pin;

  public:
    MotorDriver(int rpwm, int lpwm, int en) {
      rpwm_pin = rpwm;
      lpwm_pin = lpwm;
      en_pin = en;
    }

    void begin() {
      pinMode(rpwm_pin, OUTPUT); pinMode(lpwm_pin, OUTPUT); pinMode(en_pin, OUTPUT);
      digitalWrite(en_pin, HIGH);
    }

    // Clamped at 180 max PWM for safety headroom and traction control
    void drive(float pwm) {
      analogWrite(rpwm_pin, constrain((int)pwm, 0, 180)); analogWrite(lpwm_pin, 0);
    }
};


// ==========================================
// CLASS 5: ENCODER (tick counter)
// ==========================================
class Encoder {
  private:
    int pin_a, pin_b;
    volatile long ticks;

  public:
    Encoder(int a, int b) {
      pin_a = a;
      pin_b = b;
      ticks = 0;
    }

    void begin(void (*isr)()) {
      pinMode(pin_a, INPUT_PULLUP);
      pinMode(pin_b, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(pin_a), isr, RISING);
    }

    // Called from the interrupt
    void IRAM_ATTR onTick() {
      (digitalRead(pin_b) == LOW) ? ticks++ : ticks--;
    }

    // Returns ticks since last call and resets the counter
    long readAndReset() {
      noInterrupts();
      long t = ticks; ticks = 0;
      interrupts();
      return t;
    }
};


// ==========================================
// CLASS 6: WHEEL SPEED PID (one per wheel)
// ==========================================
class WheelSpeedPID {
  private:
    float err_sum, last_err, pwm;

  public:
    WheelSpeedPID() {
      err_sum = 0;
      last_err = 0;
      pwm = 0;
    }

    float update(float target_rpm, float actual_rpm, float dt) {
      float err = target_rpm - actual_rpm;
      err_sum += err * dt;
      float deriv = (err - last_err) / dt;
      last_err = err;
      pwm += (err * Kp_motor) + (err_sum * Ki_motor) + (deriv * Kd_motor);
      return pwm;
    }
};


// ==========================================
// CREATE OBJECTS
// ==========================================
LineSensor lineSensor;
LinePID linePID;
SteeringServo steeringServo;

MotorDriver leftMotor(L_RPWM, L_LPWM, L_EN);
MotorDriver rightMotor(R_RPWM, R_LPWM, R_EN);

Encoder leftEncoder(ENC_LEFT_A, ENC_LEFT_B);
Encoder rightEncoder(ENC_RIGHT_A, ENC_RIGHT_B);

WheelSpeedPID leftPID;
WheelSpeedPID rightPID;

// Interrupt functions to count ticks
void IRAM_ATTR leftISR()  { leftEncoder.onTick(); }
void IRAM_ATTR rightISR() { rightEncoder.onTick(); }


// ==========================================
// CLASS 7: ACKERMAN DRIVE (kinematics + speed control)
// ==========================================
class AckermanDrive {
  private:
    unsigned long last_time;

  public:
    AckermanDrive() {
      last_time = 0;
    }

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
      long t_left  = leftEncoder.readAndReset();
      long t_right = rightEncoder.readAndReset();

      // Convert ticks to real RPM
      float actual_left_rpm  = (t_left  / TICKS_PER_REV) * (60.0 / dt);
      float actual_right_rpm = (t_right / TICKS_PER_REV) * (60.0 / dt);

      // --- STAGE C: SPEED PID CONTROL ---
      float left_pwm  = leftPID.update(target_left_rpm, actual_left_rpm, dt);
      float right_pwm = rightPID.update(target_right_rpm, actual_right_rpm, dt);

      // --- STAGE D: BTS DRIVER OUTPUT ---
      leftMotor.drive(left_pwm);
      rightMotor.drive(right_pwm);
    }
};

AckermanDrive ackermanDrive;

float steering_angle = 0;
unsigned int position_val;


// ==========================================
// 8. SETUP & LOOP
// ==========================================
void setup() {
  lineSensor.begin();

  leftMotor.begin();
  rightMotor.begin();

  leftEncoder.begin(leftISR);
  rightEncoder.begin(rightISR);

  steeringServo.begin();
  Serial.begin(115200);
}

void loop() {
  // 1. Read Line Position
  position_val = lineSensor.readline();

  // 2. Line Steering PID Output
  int power_difference = linePID.compute(position_val);

  // 3. Move Front Servo and get steering angle
  steering_angle = steeringServo.servoWorking(power_difference);

  // 4. Update Rear Wheels via Ackerman Kinematics + Dual Motor PIDs
  ackermanDrive.updateWheelSpeeds(steering_angle);
}