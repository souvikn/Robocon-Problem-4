
int trig=8;
int echo=9;
int in4=4;
int in3=7;
int en1=3;
int in2=12;
int in1=13;
int en2=6;
int potvalue=0;
int d=0;
void setup()
{
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(in4, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  
}
void loop()
{
 
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long time = pulseIn(echo, HIGH);
  d = time * 0.034 / 2;         

  
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  if (d <= 10)
  {
    analogWrite(en1, 127);       
    analogWrite(en2, 127);
  }
  else
  {
    analogWrite(en1, 255);       
    analogWrite(en2, 255);
  }

  delay(100);
}
