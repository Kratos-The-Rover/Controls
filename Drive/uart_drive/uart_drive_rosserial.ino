#include <ros.h>
#include <SoftwareSerial.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#define baudrate 115200
#define vmax 0.22
#define wmax 2.84

ros::NodeHandle nh; //Node decleration
geometry_msgs::Twist twist_obj; 

float v; //recieved Lin Vel from cmd_vel
float omega; //recieved Ang vel from cmd_vel
float ang_vel; //omega mapped down to ang_vel


int rxPin =0;
int txPin = 1;
float vel_wheels[6];
int serial_pins[3] = {8,9,10};
float motorLspeed1,motorLspeed2,motorLspeed3,motorRspeed1,motorRspeed2,motorRspeed3;


SoftwareSerial MDDS1Serial=SoftwareSerial(rxPin,serial_pins[0]);
SoftwareSerial MDDS2Serial=SoftwareSerial(rxPin,serial_pins[1]);
SoftwareSerial MDDS3Serial=SoftwareSerial(rxPin,serial_pins[2]);

void sub_cb(const geometry_msgs::Twist &twist_obj)
{
  ang_vel = twist_obj.angular.z;
  omega = mymap(ang_vel , -2.84, 2.84, -0.5, 0.5);
  //if (ang_vel <= 0.05 && ang_vel >= -0.05) omega = 0;
  //else omega = map(ang_vel, -2.84, 2.84, -0.5, 0.5);
  v = twist_obj.linear.x;
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &sub_cb);

float mymap(float c,float a,float b,float d,float e)
{
  return d+(c-a)*(e-d)/(b-a);
 }
 
void setup() {
  
    nh.initNode();
    nh.subscribe(cmd_vel_sub);
 
      for ( int i=0; i<3 ;i++)
    {
        pinMode(serial_pins[i],OUTPUT);
    }
       
       MDDS1Serial.begin(baudrate);
       MDDS2Serial.begin(baudrate);
       MDDS3Serial.begin(baudrate);
       
       delay(1000);
        
       MDDS1Serial.write(0x80);
       MDDS2Serial.write(0x80);
       MDDS3Serial.write(0x80);
      delay(100);
   }
 
void loop()
{ 
  //right wheels
  vel_wheels[0] = v + omega; //back 
  vel_wheels[1] = v + omega; //middle 
  vel_wheels[2] = v + omega; //front

  // For left wheels
  vel_wheels[3] = v - omega; //back
  vel_wheels[4] = v - omega; //middle
  vel_wheels[5] = v - omega; //front

  //Converting and wrting the UART velocities to MDDS
   //for cytron 1 
   motorLspeed1 = vel_wheels[0];
   motorRspeed1 = vel_wheels[3];
   
   if(motorLspeed1>=0)
      commandbyte = 0;
   else if(motorLspeed1 <0)
      commandbyte =0x40;
      
   commandbyte = commandbyte | motorLspeed1;
   MDDS1Serial.write(commandbyte);
   
   if(motorRspeed1>=0)
      commandbyte = 0xC0;
   else if(motorRspeed1<0)
        commandbyte = 0x80;
        
   commandbyte = commandbyte | motorRspeed1;
   MDDS1Serial.write(commandbyte);
   
  Serial.println(commandbyte);
  
   //for cytron 2
   motorLspeed2 = vel_wheels[1];
   motorRspeed2 = vel_wheels[4];
   
  if(motorLspeed2>=0)
      commandbyte = 0;
   else if(motorLspeed2 <0)
      commandbyte =0x40;
      
   commandbyte = commandbyte | motorLspeed2;
   MDDS2Serial.write(commandbyte);

   if(motorRspeed2>=0)
      commandbyte = 0xC0;
   else if(motorRspeed2<0)
        commandbyte = 0x80;
        
   commandbyte = commandbyte | motorRspeed2;
   MDDS2Serial.write(commandbyte);

  Serial.println(commandbyte);
   //cytron 3
   
   motorLspeed3 = vel_wheels[2];
   motorRspeed3 = vel_wheels[5];
   
   if(motorLspeed3>=0)
      commandbyte = 0;
   else if(motorLspeed3 <0)
      commandbyte =0x40;
      
   commandbyte = commandbyte | motorLspeed1;
   MDDS3Serial.write(commandbyte);

   if(motorRspeed3>=0)
      commandbyte = 0xC0;
  else if(motorRspeed3<0)
        commandbyte = 0x80;
        
   commandbyte = commandbyte | motorRspeed3;
   MDDS3Serial.write(commandbyte);
  
    nh.spinOnce();
   
   delay(200);
   
}
