#include<SPI.h>
#include<nRF24L01.h>
#include<RF24.h>
const uint64_t pipe[2]= {0xF0F0F0F0E1LL , 0xF0F0F0F0E2LL};
RF24 radio(9,10);
int rec[1] = {2};
int red1,red2;
uint8_t pipenum[2]= {1,2};

void setup()
{
  Serial.begin(115200);
  radio.begin();
  delay(100);
  radio.setAutoAck(pipenum[0],true);
  radio.setAutoAck(pipenum[1],true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(pipenum[0],pipe[0]);
  radio.openReadingPipe(pipenum[1],pipe[1]);
  radio.startListening();
  radio.setRetries(15,15);
}
void loop()
{
  if ( radio.available(&pipenum[0]) ) 
  {
    radio.writeAckPayload( pipenum[0], rec, sizeof(int) );
    radio.read( &red1,sizeof(red1) );
    rec[0]+=2;
    Serial.print("1st integer got is : ");
    Serial.print(red1);   
  }
  
  if ( radio.available(&pipenum[1]) ) 
  {
    radio.writeAckPayload( pipenum[1], rec, sizeof(int) );
    radio.read( &red2,sizeof(red2) );
    rec[0]+=2;
    Serial.print("  \\ 2nd integer got is : ");
    Serial.println(red2);   
  }
  
}

void _print()
{
  Serial.print("1st integer got is : ");
  Serial.print(red1);
  Serial.print("  ||  2nd integer got is : ");
  Serial.println(red2);
}
