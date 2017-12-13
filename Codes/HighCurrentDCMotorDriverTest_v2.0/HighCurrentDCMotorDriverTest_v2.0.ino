#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define PWM_PIN 10
#define DIR_PIN 9
#define BRAKE_PIN 8
#define interruptPinA 2 
#define interruptPinB 3

int16_t Kp = 1, Ki = 0, Kd = 0;

float theta = 0, error = 0, old_error = 0, error_diff = 0, error_sum = 0;
float g_theta = 90.0; 

int ledPin = 13;

volatile int ticks = 0;

int count = 0;

int rot_count = 0;

int scale = 255/798, pwm = 0;

int driveMotor(int torque) 
{
  
  if (torque > 0) 
  {                                        // drive motors forward
    digitalWrite(DIR_PIN, HIGH);
    digitalWrite(BRAKE_PIN, LOW);
  } 
  else if(torque < 0)
  {                                     // drive motors backward
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(BRAKE_PIN, LOW);
    torque = abs(torque);
  }
  else if(torque == 0)
  {
     digitalWrite(BRAKE_PIN, HIGH);     
  }
  analogWrite(PWM_PIN, max(torque,0));
}

void pulse()
{
  if(digitalRead(interruptPinB) == HIGH)
  ticks++;
  else
  ticks--;
  
 // PORTB ^= (1<<PB7);
}

void timInterrupt_init()
{
    TCCR1B|=(1<<CS11)|(1<<CS10);                           // Using TIMER1 in CTC mode with OCR1A as TOP, Prescaler=1024
    TIMSK1|=(1<<OCIE1A);                                             // Enabling the Output Compare Interrupt 
    //ICR1 = 18676;    
}

/*
ISR(TIMER1_COMPA_vect)                                                // ISR 
{
   if(count > 100)  
   {	
     //Serial.print("Ticks = "); Serial.print(ticks);
     theta = (float)ticks;
     error = g_theta - theta;
     error_diff = error - old_error;
     error_sum += error;
     old_error = error;
     
     pwm = Kp*error + Ki*(error_sum) + Kd*(error_diff);
     
     PORTB^=(1<<PB7);  // Toggling PB3 and PB4 each time interrupt is called
     count = 0;
   }
   count++;
}
*/

void setup()
{
 // cli();
  
  pinMode(interruptPinA, INPUT);
  pinMode(interruptPinB, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);
  
  //digitalWrite(interruptPinA, LOW);
  
  //extInterrupt_init();
  
  //timInterrupt_init();
  
  attachInterrupt(0, pulse, RISING);
  
  sei();
  
  Serial.begin(115200);

  delay(2000);
  driveMotor(200); 
}

int i = 0;


void loop()
{
 /* if(ticks < -798)
  {
    ticks = 0;
    rot_count++;
  }
  
  if(ticks > 798)
  {
    ticks = 0;
    rot_count++;
  }
  */
  Serial.print("Theta = "); Serial.print(theta);
  Serial.print("\tError = "); Serial.print(error);
  Serial.print("\tPWM = "); Serial.print(pwm);
  Serial.print("\tTicks"); Serial.println(ticks);  
 
  //Drive_Motor(pwm);
  
  if(millis() > 10000)
  {
    driveMotors(0);
  }
  
}
