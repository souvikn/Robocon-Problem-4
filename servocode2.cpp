#include <ESP32Servo.h>
Servo servo1;
int servo_pin = 4;

int irpin1 = 13;
int irpin2 = 12;
int irpin3 = 27;
int irpin4 = 26;
int irpin5 = 25;
int irpin6 = 33;
int irpin7 = 32;
int irpin8 = 35;

int a[8];
int last_proportional = 0;
int integral = 0;
int c = 0;
float distance;

int max_angle = 30;
int min_angle = -30;
int center_angle = 90;

int i;       
int power_difference = 0;
float Kp, Ki, Kd;
unsigned int position;
int derivative, proportional;


int readline() {
  a[0] = digitalRead(irpin1);
  a[1] = digitalRead(irpin2);
  a[2] = digitalRead(irpin3);
  a[3] = digitalRead(irpin4);
  a[4] = digitalRead(irpin5);
  a[5] = digitalRead(irpin6);
  a[6] = digitalRead(irpin7);
  a[7] = digitalRead(irpin8);
  int v;
  v = (7000*a[0] + 6000*a[1] + 5000*a[2] + 4000*a[3] + 3000*a[4] + 2000*a[5] + 1000*a[6] + 0*a[7])/
      (a[0] + a[1] + a[2] + a[3] + a[4] + a[5] + a[6] + a[7]);
  return v;
}

void servoWorking(int p_difference){

  int steering_offset = p_difference;
  steering_offset = constrain(steering_offset, min_angle, max_angle);

  int target_angle = center_angle + steering_offset;
  servo1.write(target_angle);

  
}


void setup() {

  pinMode(irpin1, INPUT); 
  pinMode(irpin2, INPUT); 
  pinMode(irpin3, INPUT); 
  pinMode(irpin4, INPUT); 
  pinMode(irpin5, INPUT); 
  pinMode(irpin6, INPUT); 
  pinMode(irpin7, INPUT); 
  pinMode(irpin8, INPUT); 

  servo1.attach(servo_pin);
  Serial.begin(115200);

}

void loop() {
 
  position = readline();
  Serial.println(position);
  proportional = ((int)position - 3500);
  
  derivative = proportional - last_proportional;
  integral = integral+proportional;

  last_proportional = proportional;
  
  Kp = 0.01; 
  Ki = 0;
  Kd = 0;

  power_difference = proportional*Kp + integral*Ki + derivative*Kd;

  servoWorking(power_difference);

  
  //readline();
    
 
}