#include <ArduinoHardware.h>
#include <geometry_msgs/Twist.h>
#include <Cytron_SmartDriveDuo.h>
#include <ros.h>
#include <SoftwareSerial.h>
// #include <std_msgs/String.h>


ros::NodeHandle nh;

#define IN1 5 
#define BAUDRATE 57600
Cytron_SmartDriveDuo smartDriveDuo30(SERIAL_SIMPLFIED, IN1, BAUDRATE);

 float right_wheel=0; 
 float left_wheel=0;
 float linear=0; 
 float angular=0;

//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
//char hello[13] = "hello world!";


void callback(const geometry_msgs::Twist& msg)
{
  linear= msg.linear.x;
  angular=msg.angular.z;

  right_wheel = (linear + angular) * 100;
  left_wheel = (linear - angular) * 100;
  
}

ros::Subscriber<geometry_msgs::Twist> sub("/rover",&callback);


void setup()
{
  
  nh.initNode();
  nh.subscribe(sub); 
  // nh.advertise(chatter); 

}

void loop()
{
  // str_msg.data = hello;
  // chatter.publish( &str_msg );
  smartDriveDuo30.control(left_wheel,right_wheel);
  nh.spinOnce(); 
   

}
