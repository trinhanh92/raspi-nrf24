#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <RF24/RF24.h>

// spi device, spi speed, ce gpio pin
RF24 radio("/dev/spidev0.0",8000000,25);
 
// init counter
unsigned long count = 0;
 
void setup(void)
{
    // init radio for writing on channel 76
    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.setChannel(0x4c);
    radio.openWritingPipe(0xF0F0F0F0E1LL);
    radio.enableDynamicPayloads();
    radio.powerUp();
}
 
void loop(void)
{
    // 32 bytes is maximum payload
    char outBuffer[32]= "hello world";
 
    // pad numbers and convert to string
    // sprintf(outBuffer,"%2d",count);
 
    // transmit and increment the counter
    radio.write(outBuffer, strlen(outBuffer));
 
    // pause a second
    usleep(1000);
}

int main(int argc, char** argv) 
{
    setup();
    while(1)
        loop();
    return 0;
}
