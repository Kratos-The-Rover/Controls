
#include <Arduino.h>
#define LPWM 4
#define LEN 5
#define RPWM 6
#define REN 7

void setup() {
  pinMode(RPWM,OUTPUT);
  pinMode(LPWM,OUTPUT);
  pinMode(LEN,OUTPUT);
  pinMode(REN,OUTPUT);
  delay(1001);
}
 
 
void loop() {
  digitalWrite(REN,HIGH);
  digitalWrite(LEN,HIGH);
  analogWrite(LPWM,234);
  analogWrite(RPWM,0);
  delay(2000);
}
