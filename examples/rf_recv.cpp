#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <RF24/RF24.h>
#include <unistd.h>

using namespace std;

// spi device, spi speed, ce gpio pin
// Setup for GPIO 22 CE and CE0 CSN with SPI Speed @ 1Mhz
//RF24 radio(RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_24, BCM2835_SPI_SPEED_1MHZ);
//  RF24 radio(22, 0, BCM2835_SPI_SPEED_1MHZ);
RF24 radio(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_1MHZ);

const uint64_t addresses[2] = { 0x0504030201LL, 0x0102030405LL };
 
// init counter
unsigned long count = 0;
 
void setup(void)
{
    // init radio for writing on channel 76
    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.setChannel(0x00);
    radio.openWritingPipe(addresses[0]);  // set up address for TX
    radio.openReadingPipe(1, addresses[1]);  // set up address for RX
    radio.enableDynamicPayloads();
    radio.startListening();
    radio.powerUp();
    radio.printDetails();
}
 
void loop(void)
{
    // 32 bytes is maximum payload
    char inBuff[100]= {0};
    while(radio.available()){
        radio.read(inBuff, 100);
        printf("receiving data: %s\n", inBuff);
     }
}

int main(int argc, char** argv) 
{
    setup();
    while(1)
        loop();
    return 0;
}
