/**
 This code subscribes to a topic "final_ext" that has the stoke length to be reached. The the stoke length is changed to reach the given length
 For the base rotation, we get the input from a topic "joy", and rotate the base
 */

#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>

#define POT1 A3
#define POT2 A4

//1 is for 6 inch linear actuator, 2 is the 4 inch linear actuator, 3 is for the base rotator
//DIR1, PWM1, DIR2, PWM2, DIR3, PWM3
int pin[6] = {7,13,6,12,5,11};
//pot is the potentiometer reading. l is the stoke length(extension only). vel is the stoke velocity
float pot1,l1,vel1;
float pot2,l2,vel2;
float vel3;
//x,y is the stoke length that has to be reached. we get it from the final_ext topic. k is the right joystick input for the base rotation
float x,y,k;

const int dirPin = 2;
const int stepPin = 3;
const int stepsPerRevolution = 100;
const int left_bev_pwm_pin = 8;
const int right_bev_pwm_pin = 9;
const int left_bev_dir_pin = 24;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
const int right_bev_dir_pin = 26;

const int left_bevel_pwm = 80;
const int right_bevel_pwm = 80;

std_msgs::String mah_str;
ros::Publisher pub2("func", &mah_str);

void baseRotCw(){
  // home
  pub2.publish(&mah_str);
}

void baseRotACw(){
  // start
  pub2.publish(&mah_str);
}

void pitchUp()
{
    digitalWrite(left_bev_dir_pin,LOW); // verify
    analogWrite(left_bev_pwm_pin,left_bevel_pwm);
    digitalWrite(right_bev_dir_pin,LOW); // verify
    analogWrite(right_bev_pwm_pin,right_bevel_pwm);
    delay(500);
    analogWrite(left_bev_pwm_pin,0);
    analogWrite(right_bev_pwm_pin,0);

    mah_str.data = "pitch up";
    pub2.publish(&mah_str);
}

void pitchDown()
{
    digitalWrite(left_bev_dir_pin,LOW); // verify
    analogWrite(left_bev_pwm_pin,left_bevel_pwm);
    digitalWrite(right_bev_dir_pin,HIGH); // verify
    analogWrite(right_bev_pwm_pin,right_bevel_pwm);
    delay(500);
    analogWrite(left_bev_pwm_pin,0);
    analogWrite(right_bev_pwm_pin,0);

    mah_str.data = "pitch down";
    pub2.publish(&mah_str);
}

void gripperRotateCw() // gripper rotate clockwise
{
  digitalWrite(left_bev_dir_pin,LOW); // verify
  analogWrite(left_bev_pwm_pin,left_bevel_pwm);
  digitalWrite(right_bev_dir_pin,LOW); // verify
  analogWrite(right_bev_pwm_pin,right_bevel_pwm);
  delay(500);
  analogWrite(left_bev_pwm_pin,0);
  analogWrite(right_bev_pwm_pin,0);
  
  mah_str.data = "rotate cw";
  pub2.publish(&mah_str);
}

void gripperRotateCcw() // gripper rotate counter clockwise
{
  digitalWrite(left_bev_dir_pin,HIGH); // verify
  analogWrite(left_bev_pwm_pin,left_bevel_pwm);
  digitalWrite(right_bev_dir_pin,HIGH); // verify
  analogWrite(right_bev_pwm_pin,right_bevel_pwm);
  delay(500);
  analogWrite(left_bev_pwm_pin,0);
  analogWrite(right_bev_pwm_pin,0);

  mah_str.data = "rotate acw";
  pub2.publish(&mah_str);
}

void close()
{
  // Set motor direction clockwise
  digitalWrite(dirPin, HIGH);

  // Spin motor slowly
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(3000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(3000);
  }
  delay(100); // Wait a second 
}

void open()
{
    digitalWrite(dirPin, LOW);

  // Spin motor quickly
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(3000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(3000);
  }
  delay(100); // Wait a second  
  
}

void update_act(){
  //for linear actuator 1(6 inch)
  if(l1 > x+0.1){
    digitalWrite(pin[0],HIGH);
    vel1 = 255;
  }
  else if(l1 < x-0.1){
    digitalWrite(pin[0],LOW);
    vel1 = 255;
  }
  else{
    vel1 = 0;
  }
  //for linear actuator 2(4 inch)
  if(l2 > y+0.1){
    digitalWrite(pin[2],HIGH);
    vel2 = 255;
  }
  else if(l2 < y-0.1){
    digitalWrite(pin[2],LOW);
    vel2 = 255;
  }
  else{
    vel2 = 0;
  }
  //for base rotation
  if(k>0.8){
    digitalWrite(pin[4],LOW);
    vel3 = 255;
  }
  else if(k<-0.8){
    digitalWrite(pin[4],HIGH);
    vel3 = 255;
  }
  else{
    vel3 = 0;
  } 
}

void act_callback(const geometry_msgs::Point& msg){
  x = msg.x;
  y = msg.y;
}

void rot_callback(const sensor_msgs::Joy& msg)
{ 
  if(msg.buttons[2]==1)   //when you press x
    close();
  if(msg.buttons[1]==1)   //when you press b
    open();
  if(msg.buttons[5]==1)   // when you press RB
    gripperRotateCw();
  if(msg.buttons[4]==1)   // when you press LB 
    gripperRotateCcw();
  if(msg.buttons[6]==1)   // start
    baseRotCw();
  if(msg.buttons[7]==1)   // back
    baseRotACw();
  if(msg.buttons[0]==1)   // y
    pitchUp();
  if(msg.buttons[3]==1)   // a
    pitchDown();
}

ros::NodeHandle n;
geometry_msgs::Point c_ext;

ros::Subscriber<geometry_msgs::Point>sub1("final_ext", act_callback);
ros::Subscriber<sensor_msgs::Joy> sub2("joy", rot_callback);
ros::Publisher pub1("current_ext", &c_ext);

//coustum map fuction, as the default uses only integers
float mymap(float a, float b, float c, float d, float e){
  return a/(c-b)*(e-d);
}

void setup(){
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  for(int i=0; i<6; i++){
    pinMode(pin[i],OUTPUT);
  }
  n.initNode();
  n.advertise(pub1); 
  n.advertise(pub2); 
   
  n.subscribe(sub1);
  n.subscribe(sub2);  
}

void loop(){
  pot1 = analogRead(POT1);
  l1 = mymap(pot1,45,538,0,4);
  pot2 = analogRead(POT2);
  l2 = mymap(pot2,1,720,0,6);
  c_ext.x = l1;
  c_ext.y = l2;

  update_act();
 
  pub1.publish(&c_ext);
  analogWrite(pin[1],vel1);
  analogWrite(pin[3],vel2);
  analogWrite(pin[5],vel3);
  
  n.spinOnce();
}

