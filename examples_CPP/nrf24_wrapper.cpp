#include <cstdlib>
#include "nrf24_wrapper.h"

#ifdef __cplusplus
extern "C" {
#endif

static RF24 radio(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_1MHZ);

// void nrf24_init(uint8_t _cepin, uint8_t _cspin, uint32_t spi_speed);
void nrf24_begin() 
{
	radio.begin();
}

void nrf24_enableDynamicPayloads() 
{
	radio.enableDynamicPayloads();
}

void nrf24_setAutoAck(bool enable) 
{
	radio.setAutoAck(enable);
}

// void nrf24_setRetries(uint8_t, uint8_t)
// {

// }

// void nrf24_setDataRate(rf24_datarate_e)
// {

// }

// void nrf24_setPALevel(uint8_t)
// {

// }

// void nrf24_setChannel(uint8_t)
// {

// }

// void nrf24_setCRCLength(rf24_crclength_e)
// {

// }
// void nrf24_openWritingPipe(uint64_t);
// void nrf24_openReadingPipe(uint8_t, uint64_t);

#ifdef __cplusplus
}
#endif
