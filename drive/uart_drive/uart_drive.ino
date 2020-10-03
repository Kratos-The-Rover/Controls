
#define baudrate 115200
#define vmax 0.22
#define wmax 2.84
#define JETSerial Serial
#define MDDS1Serial Serial1
#define MDDS2Serial Serial2
#define MDDS3Serial Serial3


int rxPin =0;
int txPin = 1;
float v;
float omega;
float vel_wheels[6];
float left_wheel,right_wheel;
signed int motorLspeed1,motorLspeed2,motorLspeed3,motorRspeed1,motorRspeed2,motorRspeed3;
uint8_t commandbyte;

float mymap(float c,float a,float b,float d,float e)
{
  return d+(c-a)*(e-d)/(b-a);
 }
 
void setup() {
    
    JETSerial.begin(baudrate);
    delay(1000);
  
       MDDS1Serial.begin(baudrate);
       MDDS2Serial.begin(baudrate);
       MDDS3Serial.begin(baudrate);
        delay(1000);
       
   }
 
void loop()
{   
  //hardware serial for getting velocity from jetson to arduinomega
    if(JETSerial.available()>0)
    {
      left_wheel = myserial.read();
      right_wheel = myserial.read();
    }
  for (int i =0;i<3;i++)
  {   vel_wheels[i] = left_wheel;
  }
   for (int i =5;i>=3;i--)
  {   vel_wheels[i] = right_wheel;
  
  }
  
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
   
  JETSerial.println(commandbyte);
  
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

  JETSerial.println(commandbyte);
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

  JETSerial.println(commandbyte);
   
   
   delay(200);
   
}
