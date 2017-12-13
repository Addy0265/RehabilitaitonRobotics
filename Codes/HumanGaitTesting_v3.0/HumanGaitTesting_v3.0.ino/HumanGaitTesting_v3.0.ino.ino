#include <Wire.h>
#include <math.h>

#define PI_APP 3.14159265359
#define dt 0.1
#define G_SENS 131

const int MPU[2] = {0x68, 0x69};  // I2C address of the MPU-6050
int16_t AcX[4][2], AcY[4][2], AcZ[4][2], Tmp[4][2], GyX[4][2], GyY[4][2], GyZ[4][2], gyro_x[4][2], gyro_y[4][2], gyro_z[4][2];
const int S0 = 35, S1 = 37;

double theta_y[4][2], theta_x[4][2], theta_z[4][2], ang_x[4][2], ang_y[4][2], ang_z[4][2];
double theta_y_old[4][2] = {{0}, {0}, {0}, {0}}, theta_x_old[4][2] = {{0}, {0}, {0}, {0}}, theta_z_old[4][2] = {{0}, {0}, {0}, {0}};
double normalized_theta_x[4][2] = {{0}, {0}, {0}, {0}}, normalized_theta_y[4][2] = {{0}, {0}, {0}, {0}}, normalized_theta_z[4][2] = {{0}, {0}, {0}, {0}};

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
 else if(mux == 2)
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
      //delay(5);
   /*   if(i == 0)
        select(2);
      else if(i == 1)
        select(0);
      delay(1);
    */
  
      call(j, i);  
      
      Wire.beginTransmission(MPU[i]);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU[i],14,true);  // request a total of 14 registers
      AcX[j][i]=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
      AcY[j][i]=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ[j][i]=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Tmp[j][i]=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      GyX[j][i]=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY[j][i]=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ[j][i]=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      
      ang_x[j][i] = atan2((double)AcY[j][i], (double)AcZ[j][i])*180/PI_APP;
      ang_y[j][i] = atan2((double)AcX[j][i], (double)AcZ[j][i])*180/PI_APP;
      ang_z[j][i] = atan2((double)AcY[j][i], (double)AcX[j][i])*180/PI_APP;
      
      gyro_x[j][i] = GyX[j][i]/G_SENS;
      gyro_y[j][i] = -GyY[j][i]/G_SENS;
      gyro_z[j][i] = GyZ[j][i]/G_SENS;
      
      theta_y[j][i] =  0.85*(theta_y_old[j][i] + gyro_y[j][i]*dt) + 0.15*ang_y[j][i];
      theta_y_old[j][i] = theta_y[j][i];
      normalized_theta_y[j][i] = theta_y[j][i];
      
      theta_x[j][i] = 0.85*(theta_x_old[j][i] + gyro_x[j][i]*dt) + 0.15*ang_x[j][i];
      theta_x_old[j][i] = theta_x[j][i];
      normalized_theta_x[j][i] = theta_x[j][i];
      
      theta_z[j][i] =  0.85*(theta_z_old[j][i] + gyro_z[j][i]*dt) + 0.15*ang_z[j][i];
      theta_z_old[j][i] = theta_z[j][i];
      normalized_theta_z[j][i] = theta_z[j][i];
      
    } 
  }
}

void printData()
{
  //Serial.print("  Omega_X = "); Serial.print(gyro_x);
  //Serial.print(" | Theta_X1 = "); Serial.print(theta_x[0]);
 
  //Serial.print(" | Omega_Y = "); Serial.print(gyro_y);
  
  //Serial.print(" Theta_Y1 = "); Serial.print(normalized_theta_y[0]); 
  //Serial.print(" | Theta_Y2 = "); Serial.print(normalized_theta_y[1]);
  
  Serial.print(" L0 = "); Serial.print(normalized_theta_z[0][1]); 
  Serial.print(" L1 = "); Serial.print(normalized_theta_z[1][1]);
  Serial.print(" L2 = "); Serial.print(normalized_theta_z[2][1]); 
  Serial.print(" L3 = "); Serial.print(normalized_theta_z[3][1]);
  Serial.print("\t\t R0 = "); Serial.print(normalized_theta_z[0][0]); 
  Serial.print(" R1 = "); Serial.print(normalized_theta_z[1][0]);
  Serial.print(" R2 = "); Serial.print(normalized_theta_z[2][0]); 
  Serial.print(" R3 = "); Serial.println(normalized_theta_z[3][0]);
  
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
      
    /*  if(cnt == 0)
        select(2);
      else if(cnt == 1)
        select(0);
      delay(1);
     */
    
      call(level, cnt); 
      
      Wire.beginTransmission(MPU[cnt]);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
    }
  }
  Serial.begin(115200);
}

void loop()
{
 getMPUdata();
 printData(); 
 delay(20);
}
