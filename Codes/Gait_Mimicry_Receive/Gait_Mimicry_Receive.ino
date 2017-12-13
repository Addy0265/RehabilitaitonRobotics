#include <Servo.h> 

#define L1 8
#define L2 9
#define L3 10
#define R1 5
#define R2 6
#define R3 7

volatile byte theta_z[4][2] = {{0}, {0}};
int zeroPosition[3][2] = {{0, 0}, {0, 0}, {0, 0}};
volatile int i = -1;
uint8_t flag = 0;

Servo leftServo[3], rightServo[3];

void servoInit()
{
  leftServo[0].attach(8);
  leftServo[1].attach(9);
  leftServo[2].attach(10);
  //rightServo[0].attach(R1);
  //rightServo[1].attach(R2);
  //rightServo[2].attach(R3);
}

void servodeInit()
{
  leftServo[0].detach();
  leftServo[1].detach();
  leftServo[2].detach();
}

void driveServos()
{
  leftServo[0].write(theta_z[1][1] + zeroPosition[0][1]);
  leftServo[1].write(theta_z[1][2] + zeroPosition[1][1]);
  leftServo[2].write(theta_z[1][3] + zeroPosition[2][1]); 
 
  rightServo[0].write(theta_z[0][1] + zeroPosition[0][0]); 
  rightServo[1].write(theta_z[0][2] + zeroPosition[1][0]); 
  rightServo[2].write(theta_z[0][3] + zeroPosition[2][0]); 
}

void printData()
{
  Serial.print(" L0 = "); Serial.print((int)theta_z[0][1]); 
  Serial.print("  L1 = "); Serial.print((int)theta_z[1][1]);
  Serial.print("  L2 = "); Serial.print((int)theta_z[2][1]); 
  Serial.print("  L3 = "); Serial.print((int)theta_z[3][1]);
  Serial.print("\t R0 = "); Serial.print((int)theta_z[0][0]); 
  Serial.print(" R1 = "); Serial.print((int)theta_z[1][0]);
  Serial.print("  R2 = "); Serial.print((int)theta_z[2][0]); 
  Serial.print("  R3 = "); Serial.println((int)theta_z[3][0]); 
}

void receiveData()
{
 if (Serial1.available()) 
  {
    int inByte = (int)Serial1.read();
       
    if(i < 0)
    {
      if(inByte == 255)   // START Encountered
      i++;
    }
    else if(i < 4)
    {
      theta_z[i][1] = inByte;
      i++; 
      flag = 0; 
    }
    else if(i < 8)
    {
      theta_z[i-4][0] = inByte;
      i++; 
    }
    else
    {
      if(inByte == 254)    // END Encountered
      {
        i = -1; 
        flag = 1;
      } 
    }
  } 
}

void setup() 
{ 
  Serial.begin(38400);
  Serial1.begin(38400);
  
  servoInit();
  //leftServo[2].detach();
  //leftServo[1].detach();
  //leftServo[0].detach();
}

void loop() 
{ 
  receiveData();
 
  if(flag == 1)
  {
     printData();  
  }
  //servodeInit();
  //leftServo[2].write((int)constrain(theta_z[3][1]+50, 20, 180));
    //leftServo[1].write((int)constrain(map(theta_z[2][1], 100, 0, 20, 120), 20, 160));
    //leftServo[0].write((int)constrain(theta_z[1][1]+10, 30, 180));
    
//    leftServo[2].write((int)constrain(90-theta_z[2][1]+theta_z[3][1]+50, 20, 180));
//    leftServo[1].write((int)constrain(map(theta_z[2][1]-theta_z[1][1]+90, 90, 0, 10, 110), 0, 160));
//    leftServo[0].write((int)constrain(theta_z[1][1], 30, 180));
    
    
  leftServo[2].write(160);
  leftServo[1].write(90);
  leftServo[0].write(160);
  
}
