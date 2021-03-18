#include <ros.h>
#include <SoftwareSerial.h>
#include <controls_msgs/Rover.h>
#define baudrate 115200

ros::NodeHandle Uart_vel_command; //Node decleration
Controls_msgs::Rover Rover_obj; 
//defining variables for recieved msg here
int right_vel;
int left_vel ; 

//declaring seiral pins and cyttron serial
int rxPin =0;
int txPin = 1;
float vel_wheels[6];
int serial_pins[3] = {8,9,10};

SoftwareSerial MDDS1Serial=SoftwareSerial(rxPin,serial_pins[0]);
SoftwareSerial MDDS2Serial=SoftwareSerial(rxPin,serial_pins[1]);
SoftwareSerial MDDS3Serial=SoftwareSerial(rxPin,serial_pins[2]);

//callback for subscriber
void sub_cb( const controls_msgs::Rover &Rover_obj){
  right_vel = Rover_obj.right_wheel_vel ;
  left_vel  = Rover_obj.left_wheel_vel ;
  
}
ros::Subscriber<Controls_msgs::Rover>Rover_values_sub("/To_Arduino_msg", &sub_cb);
//Subscriber

void setup() {
  
    Uart_vel_command.initNode();
    Uart_vel_command.subscribe(Rover_values_sub);
    
 
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
     
   }
 
void loop()
{ MDDS1Serial.write(right_wheel_vel);
  MDDS1Serial.write(left_wheel_vel);

  MDDS2Serial.write(right_wheel_vel);
  MDDS2Serial.write(left_wheel_vel);

  MDDS3Serial.write(right_wheel_vel);
  MDDS3Serial.write(left_wheel_vel);
   Uart_vel_command.spinOnce();
   
   delay(200);
   
}
