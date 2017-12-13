#include <Wire.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#define PI 3.143

//IMU Variables
const int G_SENS=131;          //a division factor that directly gives omega calculated
const int MPU=0x68;                  //address of the IMU sensor
float dt = 0;
float y_old_theta=0,y_theta=0;
float x_old_theta=0,x_theta=0;
float pitch = 0, roll = 0;

int GYRY=0, GYRX=0, GYRZ=0;

//Force Sensor Variables
int fsrPin[5] = {A0, A1, A2, A3, A6};     // the FSR and 10K pulldown are connected to a0
int fsrReading[5] = {0};     // the analog reading from the FSR resistor divider
int fsrVoltage[5] = {0};     // the analog reading converted to voltage
unsigned long fsrResistance[5] = {0};  // The voltage converted to resistance, can be very big so make "long"
unsigned long fsrConductance[5] = {0}; 
long fsrForce[5] = {0};       // Finally, the resistance converted to force


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
} data;


data Data;

RF24 radio(9, 10);
const uint64_t pipes[6] = { 0xF0F0F0F0E0LL ,0xF0F0F0F0E1LL, 0xF0F0F0F0E2LL, 0xF0F0F0F0E3LL , 0xF0F0F0F0E4LL ,0xF0F0F0F0E5LL };


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
  
  //Serial.println(y_theta);
}

void get_forceSens_data()
{
   for(int i = 0; i<5; i++) {
    fsrReading[i] = analogRead(fsrPin[i]);  
//    Serial.print("SENSOR "); Serial.print(i+1); 
//    Serial.print(": Analog reading "); Serial.print(i+1); 
//    Serial.print(" = "); Serial.print(fsrReading[i]);
 
    // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
    fsrVoltage[i] = map(fsrReading[i], 0, 1023, 0, 5000);
    
    //Serial.print("\tVoltage reading in mV = ");
    //Serial.print(fsrVoltage[i]);  
   
    if (fsrVoltage[i] == 0) {
//      Serial.println("\tNo pressure");  
    } else {
      // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
      // so FSR = ((Vcc - V) * R) / V        yay math!
      fsrResistance[i] = 5000 - fsrVoltage[i];     // fsrVoltage is in millivolts so 5V = 5000mV
      fsrResistance[i] *= 10000;                // 10K resistor
      fsrResistance[i] /= fsrVoltage[i];
      //Serial.print("\tFSR resistance in ohms = ");
      //Serial.print(fsrResistance[i]);
   
      fsrConductance[i] = 1000000;           // we measure in micromhos so 
      fsrConductance[i] /= fsrResistance[i];
     // Serial.print("\tConductance in microMhos: ");
     // Serial.print(fsrConductance[i]);
   
      // Use the two FSR guide graphs to approximate the force
      if (fsrConductance[i] <= 1000) {
        fsrForce[i] = fsrConductance[i] / 80;
//        Serial.print("\tForce in Newtons: ");
//        Serial.println(fsrForce[i]);      
      } else {
        fsrForce[i] = fsrConductance[i] - 1000;
        fsrForce[i] /= 30;
//        Serial.print("\tForce in Newtons: ");
//        Serial.println(fsrForce[i]);            
      }
    }
  }
}

void setup(void) 
{
Serial.begin(115200);                                                                    //data send by IMU is too fast
Wire.begin();                                                                            //start with I2C transmission
Wire.beginTransmission(MPU);                                                                      //transmission with this address
Wire.write(0x6B);                                                                        //first specifies power management address of MCU to be given command 
Wire.write(0);                                                                           //awakes MCU by sending 0 to above register address
Wire.endTransmission(true);
 
Data.id = 2;

radio.begin();
radio.setDataRate(RF24_2MBPS);
radio.setPALevel(RF24_PA_MIN);
radio.openWritingPipe(pipes[Data.id]);
//radio.startListening();


}

void loop(void)
{  
  get_imu_data();
  
  if(Data.id == 4 || Data.id == 5)
  {
    get_forceSens_data();
  }
  
  Data.pitch = int(pitch*100);
  Data.roll = int(roll*100);
  Data.gyrx = GYRX;
  Data.gyry = GYRY;
  Data.gyrz = GYRZ;
  Data.forceSens1 = fsrForce[0];
  Data.forceSens2 = fsrForce[1];
  Data.forceSens3 = fsrForce[2];
  Data.forceSens4 = fsrForce[3];
  Data.forceSens5 = fsrForce[4];
  
  radio.write(&Data, sizeof(Data));
  
  //_print();
}

void _print()
{
  Serial.print("pitch =");Serial.print(pitch);
  Serial.print("  roll =");Serial.print(roll);
  Serial.print("  gyrx =");Serial.print(Data.gyrx);
  Serial.print("  gyry =");Serial.print(Data.gyry);
  Serial.print("  gyrz =");Serial.print(Data.gyrz);
  Serial.print("  F1 =");Serial.print(Data.forceSens1);
  Serial.print("  F2 =");Serial.print(Data.forceSens2);
  Serial.print("  F3 =");Serial.print(Data.forceSens3);
  Serial.print("  F4 =");Serial.print(Data.forceSens4);
  Serial.print("  F5 =");Serial.println(Data.forceSens5);
}
