#include <ros.h>
#include <SoftwareSerial.h>
#include <controls_msgs/Rover.h>
#define baudrate 115200

ros::NodeHandle Uart_vel_command; //Node decleration
Controls_msgs::Rover Rover_obj; 
//defining variables for recieved msg here
int right_vel;
int left_vel ; 
bool left_bev_dir
bool left_bev_pwm
bool right_bev_dir
bool right_bev_pwm
bool l1_dir
bool l1_pwm
bool l2_dir
bool l2_pwm
bool base_dir
bool base_pwm
bool stepper_dir
bool stepper_step

//declaring seiral pins and cyttron serial
int rxPin =0;
int txPin = 1;
float vel_wheels[6];
int serial_pins[3] = {8,9,10};

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
const int k =10; 
const int l1 = 293.39 * (10**-3);
const int l2 = 240.49 * (10**-3);

float arr[2] = { }

SoftwareSerial MDDS1Serial=SoftwareSerial(rxPin,serial_pins[0]);
SoftwareSerial MDDS2Serial=SoftwareSerial(rxPin,serial_pins[1]);
SoftwareSerial MDDS3Serial=SoftwareSerial(rxPin,serial_pins[2]);

//callback for subscriber
void sub_cb( const controls_msgs::Rover &Rover_obj){
  right_vel = Rover_obj.right_wheel_vel ;
  left_vel  = Rover_obj.left_wheel_vel ;
  left_bev_dir = Rover_obj.left_bev_dir ;
  if(Rover_obj.left_bev_pwm == true)
  left_bev_pwm = 80 ; 
  else 
  left_bev_pwm = 0 ;
  right_bev_dir = Rover_obj.right_bev_dir ; 
  if(Rover_obj.right_bev_pwm == true)
  right_bev_pwm = 80 ;
  else
  right_bev_pwm = 0 ;
  l1_dir = Rover_obj.l1_dir ;
  if(Rover_obj.l1_pwm == true)
  l1_pwm = k ;
  else
  l1_pwm = 0 ;
  l2_dir = Rover_obj.l2_dir ;
  if(Rover_obj.l2_pwm == true)
  l2_pwm = k ;
  else
  l2_pwm = 0 ;
  base_dir = Rover_obj.base_dir ;
  if(Rover_obj.base_pwm == true)
  base_pwm = 200 ;
  else
  base_pwm = 0 ;
  stepper_dir = Rover_obj.stepper_dir ;
  stepper_step = Rover_obj.stepper_step ;
    
}


void setup() {
  
    Uart_vel_command.initNode();
    Uart_vel_command.subscribe(Rover_values_sub);
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("chatter", 1000);
    //publisher
 
      for ( int i=0; i<3 ;i++)
    {
        pinMode(serial_pins[i],OUTPUT);
    }
       
       MDDS1Serial.begin(baudrate);
       MDDS2Serial.begin(baudrate);
       MDDS3Serial.begin(baudrate);
       
       delay(1000);
       int a = 0 ;
       int b = 128 ;
       MDDS1Serial.write(a);
       MDDS1Serial.write(b);

       MDDS2Serial.write(a);
       MDDS2Serial.write(b);
       
       MDDS3Serial.write(a);
       MDDS3Serial.write(b);
      delay(100);

      pinMode(stepPin, OUTPUT);
      pinMode(dirPin, OUTPUT);
      for(int i=0; i<6; i++){
      pinMode(pin[i],OUTPUT);
      }
     
   }
 
void loop()
{ 
  ros::Subscriber<Controls_msgs::Rover>Rover_values_sub("/To_Arduino", &sub_cb);
  //Subscriber

  
  MDDS1Serial.write(right_wheel_vel);
  MDDS1Serial.write(left_wheel_vel);

  MDDS2Serial.write(right_wheel_vel);
  MDDS2Serial.write(left_wheel_vel);

  MDDS3Serial.write(right_wheel_vel);
  MDDS3Serial.write(left_wheel_vel);
   Uart_vel_command.spinOnce();
   

  digitalWrite(pin[4], base_dir);
  analogWrite(pin[5], base_pwm);
  digitalWrite(left_bev_dir_pin, left_bev_dir); // verify
  analogWrite(left_bev_pwm_pin, left_bev_pwm);
  digitalWrite(right_bev_dir_pin, right_bev_pwm); // verify
  analogWrite(right_bev_pwm_pin, right_bevl_pwm);
  digitalWrite(pin[0], l1_dir);
  analogWrite(pin[1], l1_pwm);
  digitalWrite(pin[2], l2_dir); // verify
  analogWrite(pin[3], l3_pwm);
  digitalWrite(dirPin, stepper_dir);
  digitalWrite(stepPin, stepper_step);
  delay(200);
  int f1 = analogRead(A3);
  int f2 = analogRead(A4); // pwm from potentiometer 
  float s1= (f1/255)*l1; // feedback stroke length 
  float s2= (f2/255)*l2;
  std_msgs::Float64MultiArray array_msg;
  array_msg.data.resize(2);
  array_msg.data[1]=s1
  array_msg.data[2]=s2
  chatter_pub.publish(array_msg)

  
   
}
