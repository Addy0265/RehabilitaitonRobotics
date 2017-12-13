#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

float temp1,temp2;

RF24 radio(9, 10);
const uint64_t pipes[3] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0E2LL, 0xF0F0F0F0E3LL };

void setup(void) {
Serial.begin(9600);
radio.begin();
radio.setDataRate(RF24_250KBPS);
radio.openReadingPipe( 1 , pipes[2]);
radio.startListening();
}

void loop(void)
{
  if(radio.available())
  {
    delay(50);
    radio.read(&temp2,sizeof(temp2));
    Serial.println(temp2);
  }
  else
  {
    Serial.println("no data available");
  }
  delay(100);
//temp1 = 100;
//radio.write(&temp1, sizeof(temp1));
//delay(10);
}
