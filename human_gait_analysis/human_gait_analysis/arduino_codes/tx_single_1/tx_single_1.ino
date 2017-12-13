#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

float temp2;

RF24 radio(9, 10);
const uint64_t pipes[3] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0E2LL, 0xF0F0F0F0E3LL };

void setup(void) {
Serial.begin(9600);
radio.begin();
radio.setDataRate(RF24_250KBPS);
radio.openWritingPipe(pipes[2]);
//radio.startListening();
}

void loop(void)
{
temp2 = 150;
radio.write(&temp2, sizeof(temp2));
Serial.println("data sent");
delay(10);
}
