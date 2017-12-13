#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"



typedef struct
{
  int id; // id
  int pitch; // angle
  int roll;
  int omega;
  int forceSens1;
  int forceSens2;
  int forceSens3;
  int forceSens4;
  int forceSens5;
} data;

data temp, thigh1 ,thigh2, knee1 ,knee2, ankle1 ,ankle2;

RF24 radio(48,53);
const uint64_t pipes[6] = { 0xF0F0F0F0E0LL ,0xF0F0F0F0E1LL, 0xF0F0F0F0E2LL, 0xF0F0F0F0E3LL ,0xF0F0F0F0E4LL ,0xF0F0F0F0E5LL };


void setup(void) {
Serial.begin(115200);
radio.begin();
radio.setDataRate(RF24_2MBPS);
radio.setPALevel(RF24_PA_MIN);

radio.openReadingPipe(0, pipes[0]);
radio.openReadingPipe(1, pipes[1]);
radio.openReadingPipe(2, pipes[2]);
radio.openReadingPipe(3, pipes[3]);
radio.openReadingPipe(4, pipes[4]);
radio.openReadingPipe(5, pipes[5]);

//radio.closeReadingPipe(pipes[1]);

radio.startListening();
}

void loop(void)
{
  if ( radio.available() )
  {
  
    //delay(5);
    radio.read(&temp, sizeof(temp));
    
    switch(temp.id)
    {
      case 0:
      thigh1.id=0;
      thigh1.pitch = temp.pitch;
      thigh1.roll = temp.roll;
      thigh1.omega = temp.omega;
      thigh1.forceSens1 = 0;
      thigh1.forceSens2 = 0;
      thigh1.forceSens3 = 0;
      thigh1.forceSens4 = 0;
      thigh1.forceSens5 = 0;
      break;
      
      case 1:
      thigh2.id=1;
      thigh2.pitch = temp.pitch;
      thigh2.roll = temp.roll;
      thigh2.omega = temp.omega;
      thigh2.forceSens1 = 0;
      thigh2.forceSens2 = 0;
      thigh2.forceSens3 = 0;
      thigh2.forceSens4 = 0;
      thigh2.forceSens5 = 0;
      break;
      
      case 2:
      knee1.id=0;
      knee1.pitch = temp.pitch;
      knee1.roll = temp.roll;
      knee1.omega = temp.omega;
      knee1.forceSens1 = 0;
      knee1.forceSens2 = 0;
      knee1.forceSens3 = 0;
      knee1.forceSens4 = 0;
      knee1.forceSens5 = 0;
      break;
      
      case 3:
      knee2.id=0;
      knee2.pitch = temp.pitch;
      knee2.roll = temp.roll;
      knee2.omega = temp.omega;
      knee2.forceSens1 = 0;
      knee2.forceSens2 = 0;
      knee2.forceSens3 = 0;
      knee2.forceSens4 = 0;
      knee2.forceSens5 = 0;
      break;
      
      case 4:
      ankle1.id=4;
      ankle1.pitch=temp.pitch;
      ankle1.roll=temp.roll;
      ankle1.omega=temp.omega;
      ankle1.forceSens1 = temp.forceSens1;
      ankle1.forceSens2 = temp.forceSens2;
      ankle1.forceSens3 = temp.forceSens3;
      ankle1.forceSens4 = temp.forceSens4;
      ankle1.forceSens5 = temp.forceSens5;
      break;
      
      case 5:
      ankle2.id=5;
      ankle2.pitch=temp.pitch;
      ankle2.roll=temp.roll;
      ankle2.omega=temp.omega;
      ankle2.forceSens1 = temp.forceSens1;
      ankle2.forceSens2 = temp.forceSens2;
      ankle2.forceSens3 = temp.forceSens3;
      ankle2.forceSens4 = temp.forceSens4;
      ankle2.forceSens5 = temp.forceSens5;
      break;
      default:
      break;  
    }
     _print();
    
  }
  else
  {
    //Serial.println("no data available");
  }
  delay(1);
} 

void _print()
{
 Serial.print(" |thigh1| ");Serial.print(" pit=");Serial.print(thigh1.pitch);Serial.print(" |roll=");Serial.print(thigh1.roll);Serial.print(" |omega=");Serial.print(thigh1.omega);  
//Serial.print("\tthigh2| ");Serial.print(" pit=");Serial.print(thigh2.pitch);//Serial.print(" |roll=");Serial.print(thigh2.roll);Serial.print(" |omega=");Serial.print(thigh2.omega);
//Serial.print(" \t|knee1| ");Serial.print(" pit=");Serial.print(knee1.pitch);//Serial.print(" |roll=");Serial.print(knee1.roll);Serial.print(" |omega=");Serial.print(knee1.omega);
 Serial.print(" \t|knee2| ");Serial.print(" pit=");Serial.print(knee2.pitch);Serial.print(" |roll=");Serial.print(knee2.roll);Serial.print(" |omega=");Serial.println(knee2.omega);
//Serial.print(" \t|ankle1|");Serial.print(" pit=");Serial.print(ankle1.pitch);//Serial.print(" |roll=");Serial.print(ankle1.roll);Serial.print(" |omega=");Serial.print(ankle1.omega);Serial.print("\tF1 =");Serial.print(Data.forceSens1);Serial.print("\tF2 =");Serial.print(Data.forceSens2);Serial.print("\tF3 =");Serial.print(Data.forceSens3);Serial.print("\tF4 =");Serial.print(Data.forceSens4);Serial.print("\tF5 =");Serial.println(Data.forceSens5);
//Serial.print(" \t|ankle2| ");Serial.print(" pit=");Serial.print(ankle2.pitch);Serial.print(" |roll=");Serial.print(ankle2.roll);Serial.print(" |omega=");Serial.print(ankle2.omega);Serial.print("\tF1 =");Serial.print(ankle2.forceSens1);Serial.print("\tF2 =");Serial.print(ankle2.forceSens2);Serial.print("\tF3 =");Serial.print(ankle2.forceSens3);Serial.print("\tF4 =");Serial.print(ankle2.forceSens4);Serial.print("\tF5 =");Serial.println(ankle2.forceSens5);
}
