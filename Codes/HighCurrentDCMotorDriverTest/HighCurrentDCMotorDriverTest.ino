#define LEDPIN 13
#define DIRPIN 9
#define PWMPIN 10
#define BRAKEPIN 12 

void setPin(int pin)
{
  digitalWrite(pin, HIGH); 
}

void resetPin(int pin)
{
  digitalWrite(pin, LOW); 
}

void driveMotor(int torque)
{
  if(torque > 0)
  {
    setPin(LEDPIN);
    resetPin(DIRPIN);
    resetPin(BRAKEPIN);
    Serial.print("FORWARD\t | \t SPEED = "); Serial.println(torque);
  }
  else if(torque < 0)
  {
    resetPin(LEDPIN);
    setPin(DIRPIN);
    resetPin(BRAKEPIN);
    torque = abs(torque); 
    Serial.print("REVERSE\t | \t SPEED = "); Serial.println(torque);
  }
  else if(torque == 0)
  {
     setPin(BRAKEPIN);
     Serial.print("BRAKE\t | \t SPEED = "); Serial.println(torque);
  } 
  analogWrite(PWMPIN, torque);
}

void MotorTest(int maxSpeed)
{
  int k;
  for(k=0; k<maxSpeed; k++)
  {
    driveMotor(k);
    delay(20);
  }
  delay(2000);
  for(k=maxSpeed; k>-maxSpeed; k--)
  {
    driveMotor(k);
    delay(20);
  }
  delay(2000);
  for(k=-maxSpeed; k<0; k++)
  {
    driveMotor(k);
    delay(20);
  } 
  driveMotor(0);
}

void StallTest(int maxSpeed)
{
  int k;
  for(k=0; k<maxSpeed; k++)
  {
    driveMotor(k);
    delay(20);
  }
  delay(2000);
  driveMotor(0);
}

void setup()
{
  Serial.begin(115200);
  
  pinMode(LEDPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  pinMode(BRAKEPIN, OUTPUT);
  pinMode(PWMPIN, OUTPUT);
  
  Serial.println("Starting Motor Test..");
  delay(2000);
  
  //driveMotor(200);
  
  MotorTest(255);
  
  delay(2000);
  StallTest(255);
}

void loop()
{  
   
}
