#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#define baudrate 115200
#define vmax 0.22
#define wmax 2.84
#define MDDS1Serial Serial1
#define MDDS2Serial Serial2
#define MDDS2Serial Serial3

ros::NodeHandle nh; //Node decleration

geometry_msgs::Twist twist_obj; 
geometry_msgs::Twist pwms_obj;

float v; //recieved Lin Vel from cmd_vel
float omega; //recieved Ang vel from cmd_vel
float ang_vel; //omega mapped down to ang_vel


int rxPin =0;
int txPin = 1;
float vel_wheels[6];
float motorLspeed1,motorLspeed2,motorLspeed3,motorRspeed1,motorRspeed2,motorRspeed3;

void sub_cb(const geometry_msgs::Twist &twist_obj)
{
  ang_vel = twist_obj.angular.z;
  omega = mymap(ang_vel , -2.84, 2.84, -0.5, 0.5);
  //if (ang_vel <= 0.05 && ang_vel >= -0.05) omega = 0;
  //else omega = map(ang_vel, -2.84, 2.84, -0.5, 0.5);
  v = twist_obj.linear.x;
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &sub_cb);
ros::Publisher chatter("chatter", &twist_obj);
ros::Publisher pwms_pub("/pwms", &pwms_obj);

float mymap(float c,float a,float b,float d,float e)
{
  return d+(c-a)*(e-d)/(b-a);
 }
 
void setup() {
  
    nh.initNode();
    nh.subscribe(cmd_vel_sub);
    nh.advertise(chatter);
       
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

  twist_obj.linear.x = vel_wheels[0];
  twist_obj.linear.y = vel_wheels[1];
  twist_obj.linear.z = vel_wheels[2];
  twist_obj.angular.x = vel_wheels[3];
  twist_obj.angular.y = vel_wheels[4];
  twist_obj.angular.z = vel_wheels[5];

  //hardware serial for getting velocity from jetson to arduinomega
   //for cytron 1 
   motorLspeed1 = vel_wheels[3];
   motorRspeed1 = vel_wheels[0];
   
   if(motorLspeed1>=0)
    MDDS1Serial.write(motorLspeed1);
   if(motorRspeed1>=0);
    MDDS1Serial.write(motorRspeed1);

   //for cytron 2
   motorLspeed2 = vel_wheels[1];
   motorRspeed2 = vel_wheels[4];
   
   if(motorLspeed2>=0)
    MDDS2Serial.write(motorLspeed2);
   if(motorRspeed2>=0);
    MDDS2Serial.write(motorRspeed2);

   //cytron 3
   motorLspeed3 = vel_wheels[5];
   motorRspeed3 = vel_wheels[2];
   
   if(motorLspeed3>=0)
    MDDS3Serial.write(motorLspeed3);
   if(motorRspeed3>=0);
    MDDS3Serial.write(motorRspeed3);
    
   chatter.publish(&twist_obj);
    nh.spinOnce();
   
   delay(200);
   
}
