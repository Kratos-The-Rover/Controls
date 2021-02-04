
/*

THIS FILE CONTAINS SERVER SIDE CODE FOR FUNCTIONS IMPLEMENTED USING SERVICES

*/
#include <ros.h>
#include <Science/CustomScience.h>

/*
CustomScience.srv
============================
int operation
int direction
---
int status
============================

*/
ros::NodeHandle  nh;
using Science::CustomScience;
//https://www.arduino.cc/reference/en/libraries/stepperdriver/
#include "A4988.h"
#include <Servo.h>
//PIN DECLARATIONS BEGIN
//==========================
//stepper Motor steps per revolution
const int STEPS_PER_REV = 200;
//Change according to requirement the number of microsteps 1=full step, 2=half step etc.
int MICROSTEPS = 1;
const int STEPPER_L_STEP = 2;//check all pin nos 
const int STEPPER_L_DIR = 4;//check all pin nos 
const int STEPPER_L_ENB = 5;
// Target RPM for Left motor
const int MOTOR_L_RPM = 30;
// #define MS1 ,MS2,MS3 for left motor
const int MS1_L = 10;
const int MS2_L = 11;
const int MS3_L = 12;

const int STEPPER_R_STEP = 3;
const int STEPPER_R_DIR = 5;
const int STEPPER_R_ENB = 5;
// #define MS1 ,MS2,MS3 for right motor
const int MS1_R = 13;
const int MS2_R = 14;
const int MS3_R = 15;

// Target RPM for Right motor
const int MOTOR_R_RPM = 30;
//Target Steps for Left and Right Motor
const int MOTOR_LR_STEPS = 100;
//SERVO PWM PIN
const int servo_pwm = 6;

const int STEPPER_120_STEP = 2;//check all pin nos 
const int STEPPER_120_DIR = 4;//check all pin nos 
const int STEPPER_120_ENB = 4;//check all pin nos 
// Target RPM for 120 motor
const int MOTOR_120_RPM = 7;
// #define MS1 ,MS2,MS3 for 120 motor
const int MS1_120 = 10;
const int MS2_120 = 11;
const int MS3_120 = 12;
//SYRINGE STEPPPER MOTOR PINS
const int STEPPER_SY_STEP = 2;//check all pin nos 
const int STEPPER_SY_DIR = 4;//check all pin nos 
const int STEPPER_SY_ENB = 4;//check all pin nos 

// Target RPM for 120 motor
const int MOTOR_SY_RPM = 7;
// #define MS1 ,MS2,MS3 for 120 motor
const int MS1_SY = 10;
const int MS2_SY = 11;
const int MS3_SY = 12;
//PIN DECLARATIONS END
//==========================

void callback(CustomScience ::Request & req, CustomScience::Response & res){

int operation = req.operation;
int direction = req.direction;
    if (operation ==1)
        {
            MoveRatchet(direction);
            if(direction==-1)
                res.status = “Ratchet was lowered successfully”;
            else if(direction==1)
                res.status = “Ratchet was moved up successfully”;
        }

    else if(operation==2)
        {
            RotateServo(direction);
            if (direction = 1)
                res.status="Servo rotated from 0deg to 90deg,waiting for recall";
            else if (direction = -1)
                res.status="Servo rotated back from 90deg to 0deg,recalled";
        }


    else if(operation==3)
        {
            Rotate120();//only ckwise direction needed 
            res.status="Rotated assembly by 120 ckwise";
        
        }


    else if(operation==4)
        {
            SyringeMove(direction);
            if (direction=1)
                res.status="Syringe Fill Operation Successfully Performed";
            else if (direction = -1)
                res.status="Syringe Empty Operation Successfully Performed";
        
        }
    else
       res.status="Operation Invalid";

}






ros::ServiceServer<CustomScience::Request, CustomScience::Response> server("science_srv",&callback);


void setup()
{
  nh.initNode();
  nh.advertiseService(server);
//A4988 stepper(MOTOR_STEPS, DIR, STEP, ENABLE, MS1, MS2, MS3);
A4988 stepperL(STEPS_PER_REV, STEPPER_L_DIR, STEPPER_L_STEP, STEPPER_L_ENB,MS1_L, MS2_L, MS3_L);
A4988 stepperR(STEPS_PER_REV, STEPPER_R_DIR, STEPPER_R_STEP, STEPPER_R_ENB, MS1_R, MS2_R, MS3_R);
// Set target motor RPM  and microstepping to 1
MICROSTEPS = 1;
stepperL.begin(MOTOR_L_RPM, MICROSTEPS);
stepperR.begin(MOTOR_120_RPM, MICROSTEPS);
stepperL.setEnableActiveState(LOW);
stepperR.setEnableActiveState(LOW);
//declare servo pins
myservo.attach(servo_pwm);
//A4988 stepper(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);
A4988 stepper120(STEPS_PER_REV, STEPPER_L_DIR, STEPPER_SY_STEP, STEPPER_SY_ENB, MS1_SY, MS2_SY, MS3_SY);
Stepper120.begin(MOTOR_R_RPM, MICROSTEPS);
StepperSY.begin(MOTOR_R_RPM, MICROSTEPS);

}

void loop()
{
  nh.spinOnce();
  delay(10);
}
//FUNCTION DEFINITIONS
//====================
void MoveRatchet(int direction){
//moving steppers by predefined steps. The method is a blocking function.
//It does not return until the motor has completed stepping the specified number of steps.
//The speed and number of steps defines how long that takes.
MICROSTEPS = 1;


int steps = 1*MICROSTEPS*direction;
// energize coils - the motor will hold position
// stepper.enable();
 StepperL.enable();
 StepperR.enable();
for(int s = 0; s<MOTOR_LR_STEPS; s++)
{
 //move 1 step at a time for synchronous movement 
 StepperL.move(steps);// steps = 1 or -1
 StepperR.move(steps);
}
//// stepper.disable();
}
void RotateServo(int direction){

int angle =  45*(1 + direction); // angle = 90 if direction  = 1 and zero if direction = -1
myservo.write(angle); 
}
void Rotate120(int direction){
MICROSTEPS = 32; //1:32
Stepper120.setMicrostep(MICROSTEPS);  // Set microstep mode to 1:1
Stepper120.enable();  
     // Moving motor 120 deg 
int steps = (MICROSTEPS*STEPS_PER_REV)/3 ;         
Stepper120.move(steps);
//// stepper.disable();
}
void SyringeMove(int direction){
MICROSTEPS = 2; //1:2
StepperSY.setMicrostep(MICROSTEPS);  // Set microstep mode to 1:2
StepperSY.enable(); 
int steps = 100;
steps = steps*MICROSTEPS*direction; 
StepperSY.move(steps);
//// stepper.disable();
}
//========================