#ifndef __NRF24_WRAPPER_H_
#define __NRF24_WRAPPER_H_

#include <RF24/RF24.h>

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
// @brief init nrf24 interface
//
// @param[in] _cepin    - chip enable
// @param[in] _cspin    - chip select
// @param[in] spi_speed
// @ret		  None	
void nrf24_init(uint8_t _cepin, uint8_t _cspin, uint32_t spi_speed);
void nrf24_begin();
void nrf24_enableDynamicPayloads();
void nrf24_setAutoAck(bool);
void nrf24_setRetries(uint8_t, uint8_t);
void nrf24_setDataRate(rf24_datarate_e);
void nrf24_setPALevel(uint8_t);
void nrf24_setChannel(uint8_t);
void nrf24_setCRCLength(rf24_crclength_e);
void nrf24_openWritingPipe(uint64_t);
void nrf24_openReadingPipe(uint8_t, uint64_t);



#ifdef __cplusplus
}
#endif

#endif
