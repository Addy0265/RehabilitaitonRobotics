#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

typedef struct{
  int a; // id
  int pitch; // angle
  int roll;
  int omega;
} data;

data temp , id0 ,id1, id2 ,id3, id4 ,id5;

RF24 radio(9, 10);
const uint64_t pipes[6] = { 0xF0F0F0F0E0LL ,0xF0F0F0F0E1LL, 0xF0F0F0F0E2LL, 0xF0F0F0F0E3LL ,0xF0F0F0F0E4LL ,0xF0F0F0F0E5LL };


//uint8_t pipe[5]={ 0x0B ,0x0C ,0x0D ,0x0E ,0x0F };

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

switch(temp.a)
{
  case 0:
  id0.a=0;
  id0.pitch=temp.pitch;
  id0.roll=temp.roll;
  id0.omega=temp.omega;
  break;
  case 1:
  id1.a=1;
  id1.pitch=temp.pitch;
  id1.roll=temp.roll;
  id1.omega=temp.omega;
  break;
  case 2:
  id2.a=2;
  id2.pitch=temp.pitch;
  id2.roll=temp.roll;
  id2.omega=temp.omega;
  break;
  case 3:
  id3.a=3;
  id3.pitch=temp.pitch;
  id3.roll=temp.roll;
  id3.omega=temp.omega;
  break;
  case 4:
  id4.a=4;
  id4.pitch=temp.pitch;
  id4.roll=temp.roll;
  id4.omega=temp.omega;
  break;
  case 5:
  id5.a=5;
  id5.pitch=temp.pitch;
  id5.roll=temp.roll;
  id5.omega=temp.omega;
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
delay(5);
} 

void _print()
{
Serial.print(" |id0| ");Serial.print(" pit=");Serial.print(id0.pitch);Serial.print(" |roll=");Serial.print(id0.roll);Serial.print(" |omega=");Serial.print(id0.omega);  
Serial.print("id1| ");Serial.print(" pit=");Serial.print(id1.pitch);Serial.print(" |roll=");Serial.print(id1.roll);Serial.print(" |omega=");Serial.print(id1.omega);
Serial.print(" |id2| ");Serial.print(" pit=");Serial.print(id2.pitch);Serial.print(" |roll=");Serial.print(id2.roll);Serial.print(" |omega=");Serial.print(id2.omega);
Serial.print(" |id3| ");Serial.print(" pit=");Serial.print(id3.pitch);Serial.print(" |roll=");Serial.print(id3.roll);Serial.print(" |omega=");Serial.print(id3.omega);
Serial.print(" |id4|");Serial.print(" pit=");Serial.print(id4.pitch);Serial.print(" |roll=");Serial.print(id4.roll);Serial.print(" |omega=");Serial.print(id4.omega);
Serial.print(" |id5| ");Serial.print(" pit=");Serial.print(id5.pitch);Serial.print(" |roll=");Serial.print(id5.roll);Serial.print(" |omega=");Serial.println(id5.omega);
}
