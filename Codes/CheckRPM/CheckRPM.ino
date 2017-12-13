#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define LEDPIN 13
#define DIRPIN 8
#define BRAKEPIN 9 
#define PWMPIN 10

#define PERIOD 5000

int interruptPinA = 2, interruptPinB = 3;
volatile int RPM = 0;

volatile int ticks = 0;

void setPin(int pin)
{
  digitalWrite(pin, HIGH); 
}

void resetPin(int pin)
{
  digitalWrite(pin, LOW); 
}

void extInterruptInit()
{
   // turn on interrupts for INT4, connect Echo to INT4
  EIMSK = (1 << INT4); // enable interrupt on INT4
  EICRB = (1 << ISC40)|(1<<ISC41);      // Rising edge
}

void setOCR(int value)
{
   OCR1A = value; 
}
void setICR(int value)
{
   ICR1 = value; 
}

void timInitPWM()
{ 
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
   
  setOCR(0);     // compare match register 16MHz/256/2Hz           /// Motor Pulse given @ 10kHz
  setICR(PERIOD);
  
  TCCR1A |= (1<<COM1A1)|(1<<WGM11); //Fast PWM Mode with ICR1 as TOP, Clear on Compare Match
  TCCR1B |= (1<<WGM12)|(1<<WGM13);  
  TCCR1B |= (1<<CS11)|(1<<CS10);            // 64 prescaler 
  //TIMSK1 |= (1 << OCIE1A);          // enable timer compare interrupt
  TIMSK1 |= (1<<TOIE1);
}


ISR(INT4_vect)
{
  if(digitalRead(interruptPinB) == HIGH)
  ticks++;
  else
  ticks--;
}

ISR(TIMER1_OVF_vect)          // timer compare interrupt service routine
{
  //digitalWrite(LEDPIN, digitalRead(LEDPIN) ^ 1);   // toggle LED pin
  
  RPM = (((ticks/2000)*16000000)/(PERIOD*64))*60;
  ticks = 0;
}

void setup()
{
  noInterrupts();
  
  Serial.begin(9600);
  
  pinMode(interruptPinA, INPUT);
  pinMode(interruptPinB, INPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(DIRPIN, OUTPUT);
  pinMode(BRAKEPIN, OUTPUT);
  pinMode(PWMPIN, OUTPUT);
   
  Serial.println("Starting Motor Test..");
  //delay(1000);
  
  timInitPWM();
  extInterruptInit();
  interrupts();         // enable all(global) interrupts
  
  //driveMotor(-200);
  //delay(8000);
  //setPin(BRAKEPIN);
  //driveMotor(0);
}

void driveMotor(int torque)
{
  if(torque > 0)
  {
    setPin(LEDPIN);
    resetPin(DIRPIN);
    resetPin(BRAKEPIN);
    Serial.print("\tFORWARD\t | \t SPEED = "); Serial.println(torque);
  }
  else if(torque < 0)
  {
    resetPin(LEDPIN);
    setPin(DIRPIN);
    resetPin(BRAKEPIN);
    torque = abs(torque); 
    Serial.print("\tREVERSE\t | \t SPEED = "); Serial.println(torque);
  }
  else if(torque == 0)
  {
    setPin(BRAKEPIN);
    torque = abs(torque); 
    Serial.print("\tBRAKE\t | \t SPEED = "); Serial.println(torque); 
  }
  
  analogWrite(PWMPIN, torque);
}



void loop()
{
   Serial.print("Motor RPM = "); Serial.print(RPM); Serial.print(" RPM");
   Serial.print("\tTicks = "); Serial.print(ticks);
   driveMotor(100);
}
