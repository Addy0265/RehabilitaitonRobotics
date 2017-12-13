#include<SPI.h>
#include<nRF24L01.h>
#include<RF24.h>
int msg[1] = {1};
int rec[1] = {5};
bool stat = true;
RF24 radio(9,10);
const uint64_t pipe[2] = {0xF0F0F0F0E1LL , 0xF0F0F0F0E2LL};
uint8_t pipenum[2]={1,2};

void setup()
{
  Serial.begin(115200);
  radio.begin();
  delay(100);
  radio.setAutoAck(pipenum[0],true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.stopListening();
  radio.openWritingPipe(pipe[0]);
  radio.setRetries(15,15);
 }
void loop()
{
if(stat)
{
    if(radio.write(msg,sizeof(msg)))
    {
      Serial.print( msg[0] );
      Serial.println("...tx success");
      if(radio.isAckPayloadAvailable())
      {
        radio.read(rec,sizeof(int));
        Serial.print("received ack payload is : ");
        Serial.println(rec[0]);
      }
      else
      {
        stat = false; //doing this completely shuts down the transmitter if an ack payload is not received !!
        Serial.println("status has become false so stop here....");
      }
      msg[0]+=3;;
    if(msg[0]>=100)
    {msg[0]=1;}
    }
 }
}
