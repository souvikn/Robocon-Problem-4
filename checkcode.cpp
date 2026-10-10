#include <ESP32Servo.h>
Servo servo;
const int L_ENC_A  = 32;
const int L_ENC_B  = 33;
const int L_RPWM   = 25;
const int L_LPWM   = 26;
const int L_R_EN   = 27;
const int L_L_EN   = 14;

const int R_ENC_A  = 18;
const int R_ENC_B  = 19;
const int R_RPWM   = 16;
const int R_LPWM   = 17;
const int R_R_EN   = 21;
const int R_L_EN   = 22;

const int SERVO_PIN = 13;

const int SAMPLE_MS = 20;

const float KP = 2.0;
const float KI = 0.5;
const float KD = 0.0;
const int IR_PINS[8] = {34, 35, 36, 39, 23, 4, 5, 15};
const int LINE_LEVEL = HIGH;   
const int BASE_SPEED = 25;
const int MAX_TURN = 45;
const int STEPS = 7;
int stepSpeed[STEPS]   = {20, 35, 35, 25, 35, 30, 0};
int stepAngle[STEPS]   = {90, 90, 60, 120, 75, 90, 90};
int stepSeconds[STEPS] = {3,  4,  3,  3,   4, 2 , 2};


class Encoder {
  private:
    int pinA;
    int pinB;
    volatile long count;

  public:
    Encoder(int a, int b) {
      pinA = a;
      pinB = b;
      count = 0;
    }

    void begin() {
      pinMode(pinA, INPUT_PULLUP);
      pinMode(pinB, INPUT_PULLUP);
    }

    void handlePulse() {
      if (digitalRead(pinB) == HIGH) {
        count++;
      } else {
        count--;
      }
    }

    long getCount() {
      return count;
    }
};


class Motor {
  private:
    int rpwm;
    int lpwm;
    int rEn;
    int lEn;

  public:
    Motor(int rp, int lp, int re, int le) {
      rpwm = rp;
      lpwm = lp;
      rEn = re;
      lEn = le;
    }

    void begin() {
      pinMode(rpwm, OUTPUT);
      pinMode(lpwm, OUTPUT);
      pinMode(rEn, OUTPUT);
      pinMode(lEn, OUTPUT);
      digitalWrite(rEn, HIGH);
      digitalWrite(lEn, HIGH);
    }

    void setSpeed(int pwm) {     
      analogWrite(rpwm, pwm);
      analogWrite(lpwm, 0);
    }
};


class Steering {
  private:
    Servo servo;
    int pin;

  public:
    Steering(int p) {
      pin = p;
    }

    void begin() {
      servo.attach(pin);
      servo.write(90);
    }

    void setAngle(int angle) {
      if (angle < 45)  angle = 45;
      if (angle > 135) angle = 135;
      servo.write(angle);
    }
};


class PID {
  private:
    float kp;
    float ki;
    float kd;
    float integral;
    float lastError;

  public:
    PID(float p, float i, float d) {
      kp = p;
      ki = i;
      kd = d;
      integral = 0;
      lastError = 0;
    }

    void reset() {
      integral = 0;
      lastError = 0;
    }

    float compute(float target, float measured) {
      float error = target - measured;

      integral = integral + error;
      if (integral > 200)  integral = 200;
      if (integral < -200) integral = -200;

      float derivative = error - lastError;
      lastError = error;

      float output = (kp * error) + (ki * integral) + (kd * derivative);

      if (output > 255) output = 255;
      if (output < 0)   output = 0;
      return output;
    }
};
class LineSensors {
  private:
    int pins[8];
    int weights[8];
    int lastPosition;

  public:
    LineSensors(const int p[8]) {
      int w[8] = {-70, -50, -30, -10, 10, 30, 50, 70};
      for (int i = 0; i < 8; i++) {
        pins[i] = p[i];
        weights[i] = w[i];
      }
      lastPosition = 0;
    }

    void begin() {
      for (int i = 0; i < 8; i++) {
        pinMode(pins[i], INPUT);
      }
    }

    int getPosition() {              
      int sum = 0;
      int count = 0;

      for (int i = 0; i < 8; i++) {
        if (digitalRead(pins[i]) == LINE_LEVEL) {
          sum = sum + weights[i];
          count++;
        }
      }

      if (count > 0) {
        lastPosition = sum / count; 
      }
      return lastPosition;           
    }
};


Encoder leftEncoder(L_ENC_A, L_ENC_B);
Encoder rightEncoder(R_ENC_A, R_ENC_B);
LineSensors lineSensors(IR_PINS);
Motor leftMotor(L_RPWM, L_LPWM, L_R_EN, L_L_EN);
Motor rightMotor(R_RPWM, R_LPWM, R_R_EN, R_L_EN);

PID leftPid(KP, KI, KD);
PID rightPid(KP, KI, KD);

Steering steering(SERVO_PIN);

long leftLastCount = 0;
long rightLastCount = 0;
int currentStep = 0;
int loopsDone = 0;



void IRAM_ATTR leftEncoderISR() {
  leftEncoder.handlePulse();
}

void IRAM_ATTR rightEncoderISR() {
  rightEncoder.handlePulse();
}


void setup() {
  Serial.begin(115200);

  leftEncoder.begin();
  rightEncoder.begin();
  leftMotor.begin();
  rightMotor.begin();
  steering.begin();

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, RISING);
}

void loop() {
  
  int position = lineSensors.getPosition();            
  int targetAngle = 90 + (position * MAX_TURN) / 70;   
  int targetSpeed = BASE_SPEED;

  
  long leftNow = leftEncoder.getCount();
  long rightNow = rightEncoder.getCount();
  long leftSpeed = leftNow - leftLastCount;
  long rightSpeed = rightNow - rightLastCount;
  leftLastCount = leftNow;
  rightLastCount = rightNow;

  
  int turn = targetAngle - 90;
  int leftTarget = targetSpeed;
  int rightTarget = targetSpeed;
  if (turn < 0) leftTarget = targetSpeed - (targetSpeed * (-turn)) / 100;
  if (turn > 0) rightTarget = targetSpeed - (targetSpeed * turn) / 100;

  
  steering.setAngle(targetAngle);
  float leftPwm = leftPid.compute(leftTarget, leftSpeed);
  float rightPwm = rightPid.compute(rightTarget, rightSpeed);
  leftMotor.setSpeed((int)leftPwm);
  rightMotor.setSpeed((int)rightPwm);

  Serial.print(position);
  Serial.print(",");
  Serial.print(leftSpeed);
  Serial.print(",");
  Serial.println(rightSpeed);

  delay(SAMPLE_MS);
}