//This is the code for the backup system for drive which incorporates BTS7960 as the motor drivers
#include<ros.h>
#include <controls/msgs/Twist.h>

const int [6] = {};
const int RE_PWM[6] = {};
const int F_EN[6] = {};
const int R_EN[6] = {};

void init()
{
  for(int i=0;i<6;i++)
  {
    digitalWrite(F_EN[i],HIGH);
    digitalWrite(R_EN[i],HIGH);
    digitalWrite(F_PWM[i],0);
    digitalWrite(R_PWM[i],0);
    pinMode(FW_PWM[i],OUTPUT);
    pinMode(RW_PWM[i],OUTPUT);
    pinMode(R_EN[i],OUTPUT);
    pinMode(F_EN[i],OUTPUT);
  }  
}

void setup()
{
  Serial.begin(9600);
  init();
  Serial.println("INITIALIZED");
  nh.initNode();
  nh.subscribe(callback);
}

void callback(const controls::PWM &pwm_obj)
{
  if(pwm_obj.dir==1)
  {
    analogWrite(FW_PWM[0],pwm_obj.lf);
    analogWrite(FW_PWM[1],pwm_obj.lm);
    analogWrite(FW_PWM[2],pwm_obj.lb);
    analogWrite(FW_PWM[3],pwm_obj.rf);
    analogWrite(FW_PWM[4],pwm_obj.rm);
    analogWrite(FW_PWM[5],pwm_obj.rb);
   
  }
   if(pwm_obj.dir==0)
  {
    analogWrite(RW_PWM[0],pwm_obj.lf);
    analogWrite(RW_PWM[1],pwm_obj.lm);
    analogWrite(RW_PWM[2],pwm_obj.lb);
    analogWrite(RW_PWM[3],pwm_obj.rf);
    analogWrite(RW_PWM[4],pwm_obj.rm);
    analogWrite(RW_PWM[5],pwm_obj.rb);
  }
}
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> callback("/pwms", &sub_cb);

void loop() 
{
  nh.spinOnce();
  delay(100);
}
