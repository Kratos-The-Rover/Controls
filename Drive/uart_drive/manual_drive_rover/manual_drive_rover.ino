
#include <ArduinoHardware.h>
#include <geometry_msgs/Twist.h>
#include <Cytron_SmartDriveDuo.h>
#include <ros.h>
#include <SoftwareSerial.h>
#include <std_msgs/String.h>


ros::NodeHandle nh;

#define FRONT 5
#define MIDDLE 6
#define BACK 7 

#define BAUDRATE 57600

Cytron_SmartDriveDuo cytron_front(SERIAL_SIMPLFIED, FRONT, BAUDRATE);
Cytron_SmartDriveDuo cytron_middle(SERIAL_SIMPLFIED, MIDDLE, BAUDRATE);
Cytron_SmartDriveDuo cytron_back(SERIAL_SIMPLFIED, BACK, BAUDRATE);

 float right_wheel=0; 
 float left_wheel=0;



void callback(const geometry_msgs::Twist& msg)
{
  right_wheel= msg.linear.x;
  left_wheel=msg.linear.y;
  
}

ros::Subscriber<geometry_msgs::Twist> sub("/rover",&callback);


void setup()
{
  
  nh.initNode();
  nh.subscribe(sub);  

}

void loop()
{
  
  cytron_front.control(left_wheel,right_wheel);
  cytron_middle.control(left_wheel,right_wheel);
  cytron_back.control(left_wheel,right_wheel);
  nh.spinOnce(); 
   

}
