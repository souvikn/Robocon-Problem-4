#include <ESP32Servo.h>

const int SENSOR_PINS[5] = {13, 12, 14, 27, 26}; 
const int STEER_SERVO_PIN = 18;                   

Servo steerServo;

const int HARD_LEFT    = 150;
const int GENTLE_LEFT  = 120;
const int CENTER       = 90;
const int GENTLE_RIGHT = 60;
const int HARD_RIGHT   = 30;

void setup() {
  Serial.begin(115200); 

  
  for (int i = 0; i < 5; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
 
  steerServo.setPeriodHertz(50);
  steerServo.attach(STEER_SERVO_PIN, 500, 2400);

  
  steerServo.write(CENTER);
  
  Serial.println("--- ESP32 5-IR Line Tracker Initialized ---");
}

void loop() {
  int s[5];

  
  for (int i = 0; i < 5; i++) {
    s[i] = digitalRead(SENSOR_PINS[i]);
  }

 
  Serial.printf("Sensors: [%d %d %d %d %d] | ", s[0], s[1], s[2], s[3], s[4]);

  
  if (s[0] == 1) {
    steerServo.write(HARD_LEFT);   
    Serial.println("Steer: HARD LEFT");
  } 
  else if (s[1] == 1) {
    steerServo.write(GENTLE_LEFT); 
    Serial.println("Steer: GENTLE LEFT");
  } 
  else if (s[4] == 1) {
    steerServo.write(HARD_RIGHT);  
    Serial.println("Steer: HARD RIGHT");
  } 
  else if (s[3] == 1) {
    steerServo.write(GENTLE_RIGHT);
    Serial.println("Steer: GENTLE RIGHT");
  } 
  else if (s[2] == 1) {
    steerServo.write(CENTER);     
    Serial.println("Steer: CENTER");
  } 
  else {
    
    Serial.println("Line lost!");
  }

  delay(20); 
}