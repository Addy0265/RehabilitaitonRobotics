#include <Wire.h>
#include <math.h>

#define PI_APP 3.14159265359
#define dt 0.1
#define G_SENS 131

#define MAX_THRESH_X 40
#define MAX_THRESH_Y 40
#define MIN_THRESH_X 20
#define MIN_THRESH_Y 20

#define DUTY 150

const int MPU[2] = {0x68, 0x69};  // I2C address of the MPU-6050
int16_t AcX[2], AcY[2], AcZ[2], Tmp[2], GyX[2], GyY[2], GyZ[2], gyro_x[2], gyro_y[2];
int lMotor = 10, rMotor = 11, inA = 8, inB = 9, inC =12, inD = 13;
int accel;

double ratio[2], theta_y[2], theta_x[2], ang_x[2], ang_y[2], theta_y_old[2] = {0}, theta_x_old[2] = {0}, normalized_theta_y[2];

// theta_x - Controls Left right motion
// theta_y - Controls fwd rev motion

int fwd = 22, rev = 23, left = 24, right = 25;

void setPin(int pin)
{
  digitalWrite(pin, HIGH); 
}

void resetPin(int pin)
{
  digitalWrite(pin, LOW); 
}

void resetAll()
{
  digitalWrite(fwd, LOW);
  digitalWrite(rev, LOW);  
  digitalWrite(left, LOW); 
  digitalWrite(right, LOW); 
}

void getMPUdata()
{
  int i;
  for(i=0; i<2; i++)
  {
    Wire.beginTransmission(MPU[i]);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU[i],14,true);  // request a total of 14 registers
    AcX[i]=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    AcY[i]=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ[i]=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp[i]=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX[i]=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY[i]=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ[i]=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
    ang_x[i] = atan2((double)AcY[i], (double)AcZ[i])*180/PI_APP;
    ang_y[i] = atan2((double)AcX[i], (double)AcZ[i])*180/PI_APP;
    
    gyro_x[i] = GyX[i]/G_SENS;
    gyro_y[i] = -GyY[i]/G_SENS;
    
    theta_y[i] =  0.85*(theta_y_old[i] + gyro_y[i]*dt) + 0.15*ang_y[i];
    theta_y_old[i] = theta_y[i];
    normalized_theta_y[i] = theta_y[i];
    
    theta_x[i] = 0.85*(theta_x_old[i] + gyro_x[i]*dt) + 0.15*ang_x[i];
    theta_x_old[i] = theta_x[i];
  } 
}

void printData()
{
  //Serial.print("  Omega_X = "); Serial.print(gyro_x);
  //Serial.print(" | Theta_X1 = "); Serial.print(theta_x[0]);
 
  //Serial.print(" | Omega_Y = "); Serial.print(gyro_y);
  Serial.print(" | Theta_Y1 = "); Serial.print(normalized_theta_y[0]); 
  Serial.print(" | Theta_Y2 = "); Serial.println(normalized_theta_y[1]);
}

void setup()
{ 
  int cnt;
  Wire.begin();
  for(cnt=0; cnt<2; cnt++)
  {
    Wire.beginTransmission(MPU[cnt]);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
  }
  Serial.begin(38400);
}

void loop()
{
 getMPUdata();
 printData(); 
 //gestureRun(); 
 delay(50);
}
