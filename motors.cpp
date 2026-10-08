#define RPWM 5 //for the motor to turn clockwise
#define LPWM 13 // for the motor to turn anticlockwise
#define L_EN 14 // to enable the left side of H-Bridge of motor driver
#define R_EN 27 // to enable the right side of H-Bridge of motor driver

#define pwm_freq 20000
#define pwm_res 8
#define potpin 34

void setup()
{
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  ledcAttachChannel(RPWM, pwm_freq, pwm_res, 0); 
  ledcAttachChannel(LPWM, pwm_freq, pwm_res, 1);
}

void loop()
{
  int raw_potvalue = analogRead(potpin);
  int safe_potvalue = constrain(raw_potvalue, 0, 4095);
  int speed = map(safe_potvalue, 0, 4095, -255, 255);

  if (speed > 20) {
    ledcWrite(LPWM, 0);
    ledcWrite(RPWM, speed);
  } else if (speed < -20) {
    ledcWrite(RPWM, 0);
    ledcWrite(LPWM, abs(speed));
  } else {
    ledcWrite(LPWM, 0);
    ledcWrite(RPWM, 0);
  }
}
