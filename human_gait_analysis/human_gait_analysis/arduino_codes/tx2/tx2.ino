#include <Wire.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#define PI 3.143
const int G_SENS=131;          //a division factor that directly gives omega calculated
const int MPU=0x68;                  //address of the IMU sensor
//const float dt=0.0001;              //derivative time interval
float y_old_theta=0,y_theta=0;
float z_old_theta=0,z_theta=0;


float t1=0,t2=0,dt=0;

int GYRY=0,GYRY_old=0 , GYRZ=0;
int acc=0;

typedef struct
{
  int a; // id
  int pitch; // angle
  int roll;
  int omega;
} data;


data temp2;

RF24 radio(9, 10);
const uint64_t pipes[5] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0E2LL, 0xF0F0F0F0E3LL , 0xF0F0F0F0E4LL ,0xF0F0F0F0E5LL };


void get_imu_data()
{ t1=micros()*0.000001;
  Wire.beginTransmission(MPU);                                                        //starting the communication again
  Wire.write(0x3B);                                                                  //start with this register address (its the first data register
  Wire.endTransmission(false);                                                        //continue to read data
  Wire.requestFrom(MPU,14,true);                                                     //request the slave to send the 14 byte data
  int acc_x=Wire.read()<<8|Wire.read();                                                //acc_x is 16 bit data .the data is automatically read sequently from 0x3B
  int acc_y=Wire.read()<<8|Wire.read();                                                 //all the data is sequently stored in registers in IMU....hence we can read it sequently only by specifying the starting address .
  int acc_z=Wire.read()<<8|Wire.read();
  int tmp=Wire.read()<<8|Wire.read();                                              //each quantity has 16 bit data..however the wire.read reads only 8 bit at a time.first H register and then L register
  int gyr_x=Wire.read()<<8|Wire.read();
  int gyr_y=Wire.read()<<8|Wire.read();
  int gyr_z=Wire.read()<<8|Wire.read();
  Wire.endTransmission();                                                                  //end the transmission
  t2=micros()*0.000001;
  float ACCY=atan2((double)acc_x,(double)acc_z)*180/PI+90;
  float ACCZ=atan2((double)acc_x,(double)acc_y)*180/PI+90; 
                                                                                                       //ACCY is the theta about y axis calculated from accelerometer data
  GYRY=(float)gyr_y/G_SENS;      //GYRY is the omega about y axis          
  GYRZ=(float)gyr_z/G_SENS;                                                                                             //G_SENS is factor that directly gives the omega from raw data of gyroscope in IMU          
  dt=t2-t1;
  
  y_theta=0.85*(y_old_theta+GYRY*dt)+0.15*(ACCY);            //complimentary filter 
  z_theta=0.85*(z_old_theta+GYRZ*dt)+0.15*(ACCZ);
  
  acc=(GYRY-GYRY_old)/dt;
  GYRY_old=GYRY;
  
  y_old_theta=y_theta;
                                                              
  z_old_theta=z_theta;
  
  //Serial.println(y_theta);
}

void setup(void) 
{
Serial.begin(38400);                                                                    //data send by IMU is too fast
Wire.begin();                                                                            //start with I2C transmission
Wire.beginTransmission(MPU);                                                                      //transmission with this address
Wire.write(0x6B);                                                                        //first specifies power management address of MCU to be given command 
Wire.write(0);                                                                           //awakes MCU by sending 0 to above register address
Wire.endTransmission(true);
 

radio.begin();
radio.setDataRate(RF24_2MBPS);
radio.setPALevel(RF24_PA_MIN);
radio.openWritingPipe(pipes[2]);
//radio.startListening();

temp2.a=2;
}

void loop(void)
{  
get_imu_data();
temp2.pitch=int(y_theta);
temp2.roll=int(z_theta);
temp2.omega=GYRY;
//temp2.alpha=acc;
radio.write(&temp2, sizeof(temp2));
_print();
}

void _print()
{
  Serial.print("pitch=");Serial.print(temp2.pitch);
  Serial.print(" || roll=");Serial.print(temp2.roll);
  Serial.print(" || omega=");Serial.println(temp2.omega);
  //Serial.print(" ||alpha=");Serial.println(temp2.alpha);
}
