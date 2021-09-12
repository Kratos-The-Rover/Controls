int pwm=3;
int dir_pin=2;
boolean dir=LOW;
void setup() 
{
  pinMode(pwm,OUTPUT);
  pinMode(dir_pin,OUTPUT);  
}

void loop() 
{
  delay(1000);
  analogWrite(pwm,100);
  delay(1000);
  digitalWrite(dir_pin,dir);
  dir=!(dir);
}
