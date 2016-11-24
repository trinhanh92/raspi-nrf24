#include <string.h>
#include <RF24/nrf24.h>
#include <RF24/bcm2835.h>
#include <unistd.h>
#include <stdio.h>

const uint8_t addresses_recv[] =  {0x05, 0x04, 0x03, 0x02, 0x01};
const uint8_t addresses_trans[]  = {0x01, 0x02, 0x03, 0x04, 0x05};
 
// init counter
unsigned long count = 0;
 
void setup(void)
{
    // init radio for writing on channel 0
	// Refer to RF24.h or nRF24L01 DS for settings
	nrf24_init(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_1MHZ);
	nrf24_begin();
	nrf24_enableDynamicPayloads();
	nrf24_setAutoAck(1);
	nrf24_setRetries(15,15);
	nrf24_setDataRate(RF24_1MBPS);
	nrf24_setPALevel(RF24_PA_MAX);
	nrf24_setChannel(0x00);
	nrf24_setCRCLength(RF24_CRC_16);
	nrf24_openWritingPipe(addresses_trans);
	nrf24_openReadingPipe(1,addresses_recv);
	nrf24_startListening();

	//
	// Dump the configuration of the rf unit for debugging
	//
	nrf24_printDetails();
	
	printf("Output below : \n");
	sleep(1);
       
}

uint8_t recv_pipe = 1;
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
   
    nrf24_send(outBuffer, sizeof outBuffer);
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
