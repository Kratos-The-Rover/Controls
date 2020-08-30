#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>


//for lowering stepper motor driver A4988
const int left_enable_pin = 22;
const int right_enable_pin = 23;
const int left_step_pin = 2; // pwm i/o pin
const int left_dir_pin = 24;
const int right_step_pin = 3;// pwm i/o pin
const int right_dir_pin = 25;
const float lowering_speed = 100.0; 

// desired postion for both "lowering" steppers
long desired_position = 100;
//Left and Right Stepper Motor declaration 
AccelStepper stepperL(AccelStepper::DRIVER, left_step_pin, left_dir_pin);
AccelStepper stepperR(AccelStepper::DRIVER, right_step_pin, right_dir_pin);
// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper StepperController;


//for ratchet wheel DC driver RKI-1341 
const int ratchet_dir_pin = 26;
const int ratchet_pwm_pin = 4;
const int ratchet_brk_pin = 26;//if we require brake
const int ratchet_pwm = 200;
const unsigned long rev_time = 30000;//in milliseconds

//servo rotated 
const int servo_pwm = 5;
Servo myservo;

//DC-Submersible Pump driver L293D
const int waterpump_enable = 27;
const int waterpump_dir = 28;
const unsigned long pump_time = 10000;

//stepper controlled by joystick a4988
const int joy_step_pin = 6;
const int joy_dir_pin = 29;
const long max_steps = 1000;

//syringe stepper driver a4988
const int syr_enb_pin = 30;//syringe enable pin
const int sy_step_pin = 7;//syringe step pin
const int sy_dir_pin = 31;//syringe dir pin
AccelStepper SyringeStepper (AccelStepper::DRIVER, sy_step_pin, sy_dir_pin);

//joy stepper driver
AccelStepper JoyStepper(AccelStepper::DRIVER, joy_step_pin, joy_dir_pin);
JoyStepper.setCurrentPosition(0);
//ROtate 120 deg stepper driver a4988
const int rot_dir_pin = 32;
const int rot_step_pin = 8;
AccelStepper RotStepper(AccelStepper::DRIVER, joy_step_pin, joy_dir_pin);
//int vel;
void JoyCB(const sensor_msgs::Joy& msg)
{ 
  float x = msg.axes[0];
  if(x>0.8 && JoyStepper.currentPosition()<max_steps){
    analogWrite(joy_dir_pin,LOW);
    analogWrite(joy_step_pin,255);
  }
  else if(x<-0.8 && JoyStepper.currentPosition() > -1*max_steps){
    digitalWrite(joy_dir_pin,HIGH);
    analogWrite(joy_step_pin,255);
  }
  else{
    analogWrite(joy_step_pin,0);
  }
  

  
}

String key;

void KeyCB(const std_msgs::String &msg){
  key = msg.data;
  
  if(key == "w"){
      MoveUp();
      status.data = "Moving COllection assembly  up";
      pub_status.publish(&status);
  }
  else if (key == "s"){
      MoveDown();
      status.data = "Moving COllection assembly  down";
      pub_status.publish(&status);
  }
  else if (key == "r"){
      RotateRatchet();
      status.data = "Rotating Ratchet";
      pub_status.publish(&status);
  }
    else if (key == "e"){
      SyringeEmpty();
      status.data = "emptying syrige";
      pub_status.publish(&status);
  }
    else if (key == "f"){
      SyringeFill();
      status.data = "filling syringe";
      pub_status.publish(&status);
  }    
    else if (key == "x"){
      Rotate120();
      status.data = "Rotating cylinder by 120 deg";
      pub_status.publish(&status);
  }    
    else if (key == "z"){
      WaterPump();
      status.data = "Pumping water";
      pub_status.publish(&status);
   }   
  }  

// ros nodehandle
ros::NodeHandle n;
ros::Subscriber<sensor_msgs::Joy> JoyRead("joy", JoyCB);
ros::Subscriber<std_msgs::String> KeyRead("keys", KeyCB);

std_msgs::String status;
//will publish current operaton status.
ros::Publisher pub_status("func", &status);

void setup() {
  Serial.begin(9600);
  CollectSoilSetup();
  pinMode(waterpump_enable, OUTPUT);
  pinMode(waterpump_dir, OUTPUT);
  pinMode(joy_dir_pin, OUTPUT);
  pinMode(joy_step_pin, OUTPUT);
  pinMode()
  n.initNode();
  n.advertise(pub_status);


  n.subscribe(JoyRead);
  n.subscribe(KeyRead);
  
}

void MoveDown(){
  digitalWrite(right_enable_pin, LOW);
  digitalWrite(left_enable_pin, LOW);
  long positions[2]; // Array of desired stepper positions
  positions[0] = desired_position;
  positions[1] = desired_position;
  StepperController.moveTo(positions);
  stepperL.setSpeed(lowering_speed);
  //stepperL.setMaxSpeed(max_lowering_speed);
  
  stepperR.setSpeed(lowering_speed)
 // stepperR.setMaxSpeed(max_lowering_speed);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  
}
//  will pull-up soil collection assembly
void MoveUp() {
  long positions[2]; // Array of desired stepper positions
  positions[0] = -desired_position;
  positions[1] = -desired_position;
  StepperController.moveTo(positions);
  stepperL.setSpeed(lowering_speed);
  //stepperL.setMaxSpeed(max_lowering_speed);
  
  stepperR.setSpeed(lowering_speed)
 // stepperR.setMaxSpeed(max_lowering_speed);
  steppers.runSpeedToPosition();  
  digitalWrite(right_enable_pin, HIGH);
  digitalWrite(left_enable_pin, HIGH);
}

void CollectSoilSetup(){
  // Configure each stepper
  pinMode(right_enable_pin, OUTPUT);
  pinMode(left_enable_pin, OUTPUT);

    
  stepperL.setCurrentPosition(0);
  stepperR.setCurrentPosition(0);



//Caution: only constant speed stepper motion is supported: 
//acceleration and deceleration is not supported. All the steppers 
//managed by MultiStepper will step at a constant spped specified by setSpeed()

// Then give them to MultiStepper to manage
  StepperController.addStepper(stepperL);
  StepperController.addStepper(stepperR);

  pinMode(ratchet_brk_pin, OUTPUT);
  pinMode(ratchet_pwm_pin, OUTPUT);
  pinMode(ratchet_dir_pin, OUTPUT);
}

void RotateRatchet(){
  digitalWrite(ratchet_brk_pin, LOW);
  digitalWrite(ratchet_dir_pin, HIGH);
  analogWrite(ratchet_pwm_pin, ratchet_pwm);
  delay(rev_time);
  analogWrite(ratchet_pwm_pin, 0);
  digitalWrite(ratchet_brk_pin, HIGH);

}

void RotateServo(){
myservo.attach(servo_pwm);
delay(20);
myservo.Write(90);
delay(4000);
myservo.Write(0);
delay(20);
myservo.detach(servo_pwm);

}

void WaterPump(){
digitalWrite(waterpump_enable, HIGH);
digitalWrite(waterpump_dir, HIGH);//negative terminal grounded
delay(pump_time);
digitalWrite(waterpump_enable, LOW);
digitalWrite(waterpump_dir, LOW);
}

void SyringeFill(){
  digitalWrite(syringe_enable, LOW);
  SyringeStepper.moveTo(desired_position);
  SyringeStepper.setSpeed(lowering_speed);
  SyringeStepper.runSpeedToPosition();

}
void SyringeEmpty(){
  digitalWrite(syringe_enable, LOW);
  SyringeStepper.moveTo(-1*desired_position);
  SyringeStepper.setSpeed(lowering_speed);
  SyringeStepper.runSpeedToPosition();

}
void Rotate120(){//will rotate 120 deg ckwise relatively
  long desired_rot = 100.0;//to yield 120 deg rotation
  float rot_speed = 100.0;
  RotStepper.setCurrentPosition(0);
  RotStepper.moveTo(desired_rot);
  RotStepper.setSpeed(rot_speed);
  RotStepper.runSpeedToPosition();

}

void loop(){
  nh.spinOnce();
  delay(1);
}