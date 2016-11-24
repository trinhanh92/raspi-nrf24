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

const uint64_t addresses_trans = 0x0102030405LL;
const uint64_t addresses_recv  = 0x0102030405LL;
 
// init counter
unsigned long count = 0;
 
void setup(void)
{
    // init radio for writing on channel 0
	// Refer to RF24.h or nRF24L01 DS for settings
	radio.begin();
	radio.enableDynamicPayloads();
	radio.setAutoAck(1);
	radio.setRetries(15,15);
	radio.setDataRate(RF24_1MBPS);
	radio.setPALevel(RF24_PA_MAX);
	radio.setChannel(0x00);
	radio.setCRCLength(RF24_CRC_16);
	radio.openWritingPipe(addresses_trans);
	radio.openReadingPipe(1,addresses_recv);
	//
	// Dump the configuration of the rf unit for debugging
	//
	radio.printDetails();
	
	printf("Output below : \n");
	sleep(1);
       
}

char outBuffer[100] = {0};
void loop(void)
{
   static int cnt = 0;

    // 32 bytes is maximum payload
    memset(outBuffer,0, sizeof outBuffer);
//    cout << "transfer successful 123\n";
    // pad numbers and convert to string
     snprintf(outBuffer, sizeof outBuffer, "hello world %d", cnt);
     cnt++;
    // transmit and increment the counter
   
    radio.write(outBuffer, sizeof outBuffer);
    printf("sent %s\n", outBuffer);
    // pause a second
    sleep(1);
}

int main(int argc, char** argv) 
{
    setup();


printf("setup_main\n");
    while(1)
{

        loop();
}
    return 0;
}
