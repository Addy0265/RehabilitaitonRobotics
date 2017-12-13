#include <Wire.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define PI 3.143

//IMU Variables
const int G_SENS=131;          //a division factor that directly gives omega calculated
const int MPU=0x68;                  //address of the IMU sensor

float y_old_theta=0,y_theta=0;
float x_old_theta=0,x_theta=0;
float dt=0;

int GYRY=0, GYRX=0, GYRZ=0;
float pitch = 0, roll = 0;

typedef struct
{
  int id; // id
  int pitch; // angle
  int roll;
  int gyrx;
  int gyry;
  int gyrz;
  int forceSens1;
  int forceSens2;
  int forceSens3;
  int forceSens4;
  int forceSens5;
} data1;

typedef struct
{
  int id; // id
  float pitch; // angle
  float roll;
  int gyrx;
  int gyry;
  int gyrz;
  int forceSens1;
  int forceSens2;
  int forceSens3;
  int forceSens4;
  int forceSens5;
} data2;

data1 temp; 
data2 thigh1 ,thigh2, knee1 ,knee2, ankle1 ,ankle2;

RF24 radio(48,53);
const uint64_t pipes[6] = { 0xF0F0F0F0E0LL ,0xF0F0F0F0E1LL, 0xF0F0F0F0E2LL, 0xF0F0F0F0E3LL ,0xF0F0F0F0E4LL ,0xF0F0F0F0E5LL };


void get_imu_data()
{ 
  Wire.beginTransmission(MPU);                                                        //starting the communication again
  Wire.write(0x3B);                                                                  //start with this register address (its the first data register
  Wire.endTransmission(false);                                                        //continue to read data
  Wire.requestFrom(MPU,14,true);                                                     //request the slave to send the 14 byte data
  
  int16_t acc_x=Wire.read()<<8|Wire.read();                                                //acc_x is 16 bit data .the data is automatically read sequently from 0x3B
  int16_t acc_y=Wire.read()<<8|Wire.read();                                                 //all the data is sequently stored in registers in IMU....hence we can read it sequently only by specifying the starting address .
  int16_t acc_z=Wire.read()<<8|Wire.read();
  int16_t tmp=Wire.read()<<8|Wire.read();                                              //each quantity has 16 bit data..however the wire.read reads only 8 bit at a time.first H register and then L register
  int16_t gyr_x=Wire.read()<<8|Wire.read();
  int16_t gyr_y=Wire.read()<<8|Wire.read();
  int16_t gyr_z=Wire.read()<<8|Wire.read();
  Wire.endTransmission();                                                                  //end the transmission
  
  double ACCY=atan2((double)acc_x,(double)acc_z)*180/PI+90;
  double ACCX=atan2((double)acc_y,(double)acc_z)*180/PI+90; 
                                                                                                       //ACCY is the theta about y axis calculated from accelerometer data
  GYRX=(float)gyr_x/G_SENS;
  GYRY=-(float)gyr_y/G_SENS;      //GYRY is the omega about y axis          
  GYRZ=(float)gyr_z/G_SENS;                                                                                             //G_SENS is factor that directly gives the omega from raw data of gyroscope in IMU          
  dt=0.01;
  
  y_theta=0.85*(y_old_theta+GYRY*dt)+0.15*(ACCY);            //complimentary filter 
  x_theta=0.85*(x_old_theta+GYRX*dt)+0.15*(ACCX);
   
  y_old_theta=y_theta;
  x_old_theta=x_theta;

  pitch = y_theta;
  roll = x_theta;

  if(pitch>180 || pitch<-180)
  {
    if(pitch>180)
    {
      pitch = pitch-360;
    }
    else if(pitch<-180)
    {
      pitch = pitch+360;
    }
    else 
      pitch = 180;
  }

  if(roll>180 || roll<-180)
  {
    if(roll>180)
    {
      roll = roll-360;
    }
    else if(roll<-180)
    {
      roll = roll+360;
    }
    else 
      roll = 180;
  }
}

void setup(void) {
Serial.begin(115200);

Wire.begin();                                                                            //start with I2C transmission
Wire.beginTransmission(MPU);                                                                      //transmission with this address
Wire.write(0x6B);                                                                        //first specifies power management address of MCU to be given command 
Wire.write(0);                                                                           //awakes MCU by sending 0 to above register address
Wire.endTransmission(true);

radio.begin();
radio.setDataRate(RF24_2MBPS);
radio.setPALevel(RF24_PA_MIN);

radio.openReadingPipe(0, pipes[0]);
radio.openReadingPipe(1, pipes[1]);
radio.openReadingPipe(2, pipes[2]);
radio.openReadingPipe(3, pipes[3]);
radio.openReadingPipe(4, pipes[4]);
radio.openReadingPipe(5, pipes[5]);

//radio.closeReadingPipe(pipes[1]);

radio.startListening();
}

void loop(void)
{
  get_imu_data();
  
  if ( radio.available() )
  {
    //delay(5);
    radio.read(&temp, sizeof(temp));
    
    switch(temp.id)
    {
      case 0:
      thigh1.id=0;
      thigh1.pitch = ((float)temp.pitch)/100;
      thigh1.roll = ((float)temp.roll)/100;
      thigh1.gyrx = temp.gyrx;
      thigh1.gyry = temp.gyry;
      thigh1.gyrz = temp.gyrz;
      thigh1.forceSens1 = 0;
      thigh1.forceSens2 = 0;
      thigh1.forceSens3 = 0;
      thigh1.forceSens4 = 0;
      thigh1.forceSens5 = 0;
      break;
      
      case 1:
      thigh2.id=1;
      thigh2.pitch = ((float)temp.pitch)/100;
      thigh2.roll = ((float)temp.roll)/100;
      thigh2.gyrx = temp.gyrx;
      thigh2.gyry = temp.gyry;
      thigh2.gyrz = temp.gyrz;
      thigh2.forceSens1 = 0;
      thigh2.forceSens2 = 0;
      thigh2.forceSens3 = 0;
      thigh2.forceSens4 = 0;
      thigh2.forceSens5 = 0;
      break;
      
      case 2:
      knee1.id=0;
      knee1.pitch = ((float)temp.pitch)/100;
      knee1.roll = ((float)temp.roll)/100;
      knee1.gyrx = temp.gyrx;
      knee1.gyry = temp.gyry;
      knee1.gyrz = temp.gyrz;
      knee1.forceSens1 = 0;
      knee1.forceSens2 = 0;
      knee1.forceSens3 = 0;
      knee1.forceSens4 = 0;
      knee1.forceSens5 = 0;
      break;
      
      case 3:
      knee2.id=0;
      knee2.pitch = ((float)temp.pitch)/100;
      knee2.roll = ((float)temp.roll)/100;
      knee2.gyrx = temp.gyrx;
      knee2.gyry = temp.gyry;
      knee2.gyrz = temp.gyrz;
      knee2.forceSens1 = 0;
      knee2.forceSens2 = 0;
      knee2.forceSens3 = 0;
      knee2.forceSens4 = 0;
      knee2.forceSens5 = 0;
      break;
      
      case 4:
      ankle1.id=4;
      ankle1.pitch = ((float)temp.pitch)/100;
      ankle1.roll = ((float)temp.roll)/100;
      ankle1.gyrx = temp.gyrx;
      ankle1.gyry = temp.gyry;
      ankle1.gyrz = temp.gyrz;
      ankle1.forceSens1 = temp.forceSens1;
      ankle1.forceSens2 = temp.forceSens2;
      ankle1.forceSens3 = temp.forceSens3;
      ankle1.forceSens4 = temp.forceSens4;
      ankle1.forceSens5 = temp.forceSens5;
      break;
      
      case 5:
      ankle2.id=5;
      ankle2.pitch = ((float)temp.pitch)/100;
      ankle2.roll = ((float)temp.roll)/100;
      ankle2.gyrx = temp.gyrx;
      ankle2.gyry = temp.gyry;
      ankle2.gyrz = temp.gyrz;
      ankle2.forceSens1 = temp.forceSens1;
      ankle2.forceSens2 = temp.forceSens2;
      ankle2.forceSens3 = temp.forceSens3;
      ankle2.forceSens4 = temp.forceSens4;
      ankle2.forceSens5 = temp.forceSens5;
      break;
      default:
      break;  
    }  
  }
  else
  {
    //Serial.println("no data available");
  }
  _print();
  //delay(1);
} 

void _print()
{
 Serial.print("t1: ");Serial.print(" pit=");Serial.print(thigh1.pitch);Serial.print(" roll=");Serial.print(thigh1.roll);  
 Serial.print("  t2: ");Serial.print(" pit=");Serial.print(thigh2.pitch);Serial.print(" roll=");Serial.print(thigh2.roll);
 Serial.print("  k1: ");Serial.print(" pit=");Serial.print(knee1.pitch);Serial.print(" roll=");Serial.print(knee1.roll);
 Serial.print("  k2: ");Serial.print(" pit=");Serial.print(knee2.pitch);Serial.print(" roll=");Serial.print(knee2.roll);
 Serial.print("  a1: ");Serial.print(" pit=");Serial.print(ankle1.pitch);Serial.print(" roll=");Serial.print(ankle1.roll);//Serial.print("\tF1 =");Serial.print(Data.forceSens1);Serial.print("\tF2 =");Serial.print(Data.forceSens2);Serial.print("\tF3 =");Serial.print(Data.forceSens3);Serial.print("\tF4 =");Serial.print(Data.forceSens4);Serial.print("\tF5 =");Serial.println(Data.forceSens5);
 Serial.print("  a2: ");Serial.print(" pit=");Serial.print(ankle2.pitch);Serial.print(" roll=");Serial.print(ankle2.roll);//Serial.print("\tF1 =");Serial.print(ankle2.forceSens1);Serial.print("\tF2 =");Serial.print(ankle2.forceSens2);Serial.print("\tF3 =");Serial.print(ankle2.forceSens3);Serial.print("\tF4 =");Serial.print(ankle2.forceSens4);Serial.print("\tF5 =");Serial.println(ankle2.forceSens5);
 Serial.print("  T: ");Serial.print(" pit=");Serial.print(pitch);Serial.print(" roll=");Serial.println(roll);
}
