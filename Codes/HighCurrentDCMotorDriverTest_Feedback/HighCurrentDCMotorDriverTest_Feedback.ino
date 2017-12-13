#define RESOLUTION 497
#define PERIOD 0.26214 // in seconds

#define LEDPIN 13
#define DIRPIN 8
#define BRAKEPIN 9 
#define PWMPIN 10
#define interruptPinA 2 
#define interruptPinB 3

float Kp = 1, Ki = 0, Kd = 0;
float theta = 0, error = 0, old_error = 0, error_diff = 0, error_sum = 0;
float g_theta = 90.0; 
volatile int ticks = 0;
volatile int count = 0;
volatile int rot_count = 0;
int scale = 255.0/RESOLUTION, pwm = 0;
volatile int rpm = 0;

void setPin(int pin)
{
  digitalWrite(pin, HIGH); 
}

void resetPin(int pin)
{
  digitalWrite(pin, LOW); 
}

void tim3Init()
{
  TCCR3A = 0;
  TCCR3B = 0;
  
  TCCR3B |= (1<<CS31)|(1<<CS30); // select internal clock with no prescaling
  TCNT3 = 0; // reset counter to zero
  TIMSK3 = 1<<TOIE3; // enable timer interrupt 
}

void driveMotor(int torque)
{
  if(torque > 0)
  {
    setPin(LEDPIN);
    resetPin(DIRPIN);
    resetPin(BRAKEPIN);
    //Serial.print("FORWARD\t | \t SPEED = "); Serial.println(torque);
  }
  else if(torque < 0)
  {
    resetPin(LEDPIN);
    setPin(DIRPIN);
    resetPin(BRAKEPIN);
    torque = abs(torque); 
   // Serial.print("REVERSE\t | \t SPEED = "); Serial.println(torque);
  }
  else if(torque == 0)
  {
     setPin(BRAKEPIN);
     //Serial.print("BRAKE\t | \t SPEED = "); Serial.println(torque);
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
  delay(5000);
  driveMotor(0);
}

void pulse()
{
  if(digitalRead(interruptPinB) == HIGH)
  ticks++;
  else
  ticks--;
  
 // _print();
  //Serial.print("Ticks"); Serial.println(ticks); 
 // PORTB ^= (1<<PB7);
}

ISR(TIMER3_OVF_vect)
{
  rpm = ticks/(PERIOD*RESOLUTION)*60;
  ticks = 0;
  
  if(count == 5)
  {
    PORTB ^= (1<<PB7);
    count = 0;
  }
  count++;
  _print();
}

void _print()
{
 // Serial.print("Theta = "); Serial.print(theta);
  //Serial.print("\tError = "); Serial.print(error);
  //Serial.print("\tPWM = "); Serial.print(pwm);
  Serial.print("\tTicks = "); Serial.print(ticks);  
  Serial.print("\tRPM = "); Serial.println(rpm); 
}

void setup()
{
  
  noInterrupts();
  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  pinMode(BRAKEPIN, OUTPUT);
  pinMode(PWMPIN, OUTPUT);
  
  attachInterrupt(0, pulse, RISING);
  interrupts();
  tim3Init();
  Serial.println("Starting Motor Test..");
  delay(2000);
  
  //MotorTest(255);
  
  delay(1000);
  StallTest(255);
}

void loop()
{  
   
}
