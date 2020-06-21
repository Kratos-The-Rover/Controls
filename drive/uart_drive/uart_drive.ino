
#include <SoftwareSerial.h>
#define baudrate 115200
#define vmax 0.22
#define wmax 2.84
int rxPin =0;
int txPin = 1;
float v;
float omega;
float vel_wheels[6];
int serial_pins[3] = {8,9,10};
float motorLspeed1,motorLspeed2,motorLspeed3,motorRspeed1,motorRspeed2,motorRspeed3;

SoftwareSerial myserial=SoftwareSerial(rxPin,txPin);
SoftwareSerial MDDS1Serial=SoftwareSerial(rxPin,serial_pins[0]);
SoftwareSerial MDDS2Serial=SoftwareSerial(rxPin,serial_pins[1]);
SoftwareSerial MDDS3Serial=SoftwareSerial(rxPin,serial_pins[2]);

float mymap(float c,float a,float b,float d,float e)
{
  return d+(c-a)*(e-d)/(b-a);
 }
 
void setup() {
    
    myserial.begin(baudrate);
    delay(1000);
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
   }
 
void loop()
{   
  //hardware serial for getting velocity from jetson to arduinomega
  for (int i =0;i<6;i++)
  {
    if(myserial.available()>0)
    {
    vel_wheels[i] = myserial.read();
    vel_wheels[i]=mymap(vel_wheels[i],-(0.5+vmax),(0.5+vmax),0,255);
    }
  }
   //for cytron 1 
   motorLspeed1 = vel_wheels[0];
   motorRspeed1 = vel_wheels[1];
   
   if(motorLspeed1>=0)
    MDDS1Serial.write(motorLspeed1);
   if(motorRspeed1>=0);
    MDDS1Serial.write(motorRspeed1);

   //for cytron 2
   motorLspeed2 = vel_wheels[2];
   motorRspeed2 = vel_wheels[3];
   
   if(motorLspeed2>=0)
    MDDS2Serial.write(motorLspeed2);
   if(motorRspeed2>=0);
    MDDS2Serial.write(motorRspeed2);

   //cytron 3
   motorLspeed3 = vel_wheels[4];
   motorRspeed3 = vel_wheels[5];
   
   if(motorLspeed3>=0)
    MDDS3Serial.write(motorLspeed3);
   if(motorRspeed3>=0);
    MDDS3Serial.write(motorRspeed3);
   
   
   delay(200);
   
}
