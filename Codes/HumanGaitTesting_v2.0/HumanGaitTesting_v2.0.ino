#include <Wire.h>
#include <math.h>

#define PI_APP 3.14159265359
#define dt 0.1
#define G_SENS 131

const int MPU[2] = {0x68, 0x69};  // I2C address of the MPU-6050
const int S0 = 35, S1 = 37;

int16_t AcX[2][4], AcY[2][4], AcZ[2][4], Tmp[2][4], GyX[2][4], GyY[2][4], GyZ[2][4], gyro_x[2][4], gyro_y[2][4], gyro_z[2][4];
double theta_y[2][4], theta_x[2][4], theta_z[2][4], ang_x[2][4], ang_y[2][4], ang_z[2][4];
double theta_y_old[2][4] = {{0}, {0}}, theta_x_old[2][4] = {{0}, {0}}, theta_z_old[2][4] = {{0}, {0}};
double normalized_theta_x[2][4] = {{0}, {0}}, normalized_theta_y[2][4] = {{0}, {0}}, normalized_theta_z[2][4] = {{0}, {0}};

void select(int mux)
{
 if(mux == 0)
 {
  resetPin(S1);
  resetPin(S0);
 } 
 else if(mux == 1)
 {
  resetPin(S1);
  setPin(S0);
 }
 else if(mux == 4)
 {
  setPin(S1);
  resetPin(S0);
 }
 else if(mux == 3)
 {
  setPin(S1);
  setPin(S0);
 }
}

void setPin(int pin)
{
  digitalWrite(pin, HIGH); 
}

void resetPin(int pin)
{
  digitalWrite(pin, LOW); 
}

void call(int level, int i)
{
  if(level == 0)
  {
    if(i == 0)
      select(2);
    else if(i == 1)
      select(0);
    delayMicroseconds(500);    
  }
  else if(level == 1)
  {
    if(i == 0)
      select(1);
    else if(i == 1)
      select(2);
    delayMicroseconds(500);    
  }
  else if(level == 2)
  {
    if(i == 0)
      select(0);
    else if(i == 1)
      select(3);
    delayMicroseconds(500);    
  }
  else if(level == 3)
  {
    if(i == 0)
      select(3);
    else if(i == 1)
      select(1);
    delayMicroseconds(500);    
  }
}

void getMPUdata()
{
  int i, j;
  
  for(j=0; j<4; j++)
  {
      for(i=0; i<2; i++)
      {
        call(j, i);
       
        Wire.beginTransmission(MPU[i]);
        Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU[i],14,true);  // request a total of 14 registers
        AcX[i][j]=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
        AcY[i][j]=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        AcZ[i][j]=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        Tmp[i][j]=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
        GyX[i][j]=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        GyY[i][j]=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        GyZ[i][j]=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
        
        ang_x[i][j] = atan2((double)AcY[i][j], (double)AcZ[i][j])*180/PI_APP;
        ang_y[i][j] = atan2((double)AcX[i][j], (double)AcZ[i][j])*180/PI_APP;
        ang_z[i][j] = atan2((double)AcY[i][j], (double)AcX[i][j])*180/PI_APP;
        
        gyro_x[i][j] = GyX[i][j]/G_SENS;
        gyro_y[i][j] = -GyY[i][j]/G_SENS;
        gyro_z[i][j] = GyZ[i][j]/G_SENS;
        
        theta_y[i][j] =  0.85*(theta_y_old[i][j] + gyro_y[i][j]*dt) + 0.15*ang_y[i][j];
        theta_y_old[i][j] = theta_y[i][j];
        normalized_theta_y[i][j] = theta_y[i][j];
        
        theta_x[i][j] = 0.85*(theta_x_old[i][j] + gyro_x[i][j]*dt) + 0.15*ang_x[i][j];
        theta_x_old[i][j] = theta_x[i][j];
        normalized_theta_x[i][j] = theta_x[i][j];
        
        theta_z[i][j] =  0.85*(theta_z_old[i][j] + gyro_z[i][j]*dt) + 0.15*ang_z[i][j];
        theta_z_old[i][j] = theta_z[i][j];
        normalized_theta_z[i][j] = theta_z[i][j];
        
      } 
  }
}

void printData()
{
  //Serial.print("  Omega_X = "); Serial.print(gyro_x);
  //Serial.print(" | Theta_X1 = "); Serial.print(theta_x[0]);
 
  //Serial.print(" | Omega_Y = "); Serial.print(gyro_y);
  
  //Serial.print(" Theta_Y1 = "); Serial.print(normalized_theta_y[0][0]); 
  //Serial.print(" | Theta_Y2 = "); Serial.print(normalized_theta_y[1][0]);
  
  Serial.print(" Theta_Z1 = "); Serial.print(normalized_theta_z[0][1]); 
  Serial.print(" | Theta_Z2 = "); Serial.println(normalized_theta_z[1][1]);
}

void setup()
{ 
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  
  //select(0);
  
  int cnt, level;
  Wire.begin();
  
  for(level=0; level<4; level++)
  {
    for(cnt=0; cnt<2; cnt++)
    {
      call(level, cnt);
      
      Wire.beginTransmission(MPU[cnt]);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
    }
  } 
  call(0,0);
  
  Serial.begin(115200);
}

void loop()
{
 getMPUdata();
 printData(); 
 delay(20);
}
