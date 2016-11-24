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

const uint64_t addresses[2] = { 0xABCDABCD71LL, 0x0102030405LL };
 
// init counter
unsigned long count = 0;
 
void setup(void)
{
    // init radio for writing on channel 76
    cout << "setup begin\n";
    radio.begin();
    cout << "setup PA level\n";
    radio.setPALevel(RF24_PA_MAX);
    cout << "setup channel\n";
    radio.setChannel(0x00);
    cout << "setup send pipe\n";
    radio.openWritingPipe(addresses[0]);  // set up address for TX
    radio.openReadingPipe(1, addresses[1]);  // set up address for RX
    cout << "setup dynamic payload\n";
    radio.enableDynamicPayloads();
    cout << "setup powerup\n";
    radio.powerUp();
    cout << "start startListening\n";
    radio.startListening();
    radio.printDetails();
}
 
void loop(void)
{
    // 32 bytes is maximum payload
    char inBuff[100]= {0};
    while(radio.available()){
        radio.read(&inBuff, 100);
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
