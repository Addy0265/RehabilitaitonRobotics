#include <Wire.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#define PI 3.143
const int G_SENS=131;          //a division factor that directly gives omega calculated
const int MPU=0x68;                  //address of the IMU sensor
const float dt=0.0001;              //derivative time interval
float y_old_theta=0,y_theta=0;       float y_sum_theta=0;float y_diff;

typedef struct{
  int a; // id
  int b; // angle
} data;


data temp3;

RF24 radio(9, 10);
const uint64_t pipes[5] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0E2LL, 0xF0F0F0F0E3LL , 0xF0F0F0F0E4LL ,0xF0F0F0F0E5LL };

uint8_t pipe[5]={ 0x0B ,0x0C ,0x0D ,0x0E ,0x0F };

void get_imu_data()
{
  Wire.beginTransmission(MPU); //starting the communication again
  Wire.write(0x3B);  //start with this register address (its the first data register
  Wire.endTransmission(false);  //continue to read data
  Wire.requestFrom(MPU,14,true);  //request the slave to send the 14 byte data
  int acc_x=Wire.read()<<8|Wire.read();  //acc_x is 16 bit data .the data is automatically read sequently from 0x3B
  int acc_y=Wire.read()<<8|Wire.read();  //all the data is sequently stored in registers in IMU....hence we can read it sequently only by specifying the starting address .
  int acc_z=Wire.read()<<8|Wire.read();
  int tmp=Wire.read()<<8|Wire.read();  //each quantity has 16 bit data..however the wire.read reads only 8 bit at a time.first H register and then L register
  int gyr_x=Wire.read()<<8|Wire.read();
  int gyr_y=Wire.read()<<8|Wire.read();
  int gyr_z=Wire.read()<<8|Wire.read();
  Wire.endTransmission();  //end the transmission
  float ACCY=atan2((double)acc_x,(double)acc_z)*180/PI;      //ACCY is the theta about y axis calculated from accelerometer data
  int GYRY=(float)gyr_y/G_SENS;          //GYRY is the omega about y axis          
  //G_SENS is factor that directly gives the omega from raw data of gyroscope in IMU          
  
  y_theta=0.85*(y_old_theta+GYRY*dt)+0.15*(ACCY);//complimentary filter 
  y_diff=y_theta-y_old_theta; 
  y_sum_theta+=y_theta;                                                              
  y_old_theta=y_theta;
  
  //Serial.println(y_theta);
}

void setup(void) {
Serial.begin(38400);            //data send by IMU is too fast
Wire.begin();   //start with I2C transmission
Wire.beginTransmission(MPU);  //transmission with this address
Wire.write(0x6B);  //first specifies power management address of MCU to be given command 
Wire.write(0);   //awakes MCU by sending 0 to above register address
Wire.endTransmission(true);
 

radio.begin();
radio.setDataRate(RF24_2MBPS);
radio.setPALevel(RF24_PA_MIN);
radio.openWritingPipe(pipes[3]);
radio.startListening();

temp3.a=3;
}

void loop(void)
{  
get_imu_data();
temp3.b=int(y_theta);
//temp1 = "ID1="+String(y_theta);
Serial.println(temp3.b);
//temp1=y_theta;
radio.write(&temp3, sizeof(temp3));
}
