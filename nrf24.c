#include "./RF24_config.h"
#include "./nrf24.h"
#include "./nRF24L01.h"

static uint8_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
static uint8_t csn_pin; /**< SPI Chip select */
static uint16_t spi_speed; /**< SPI Bus Speed */
// static bool wide_band; /* 2Mbs data rate in use? */
static bool p_variant; /* False for RF24L01 and TRUE for RF24L01P */
static uint8_t payload_size; /**< Fixed size of payloads */

static bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
static uint8_t pipe0_reading_address[ADDR_WIDTH]; /**< Last address set on pipe 0 for reading. */
static uint8_t addr_width;

static uint8_t debug ; /* Debug flag */
static uint8_t spi_rxbuff[32+1] ; //SPI receive buffer (payload max 32 bytes)
static uint8_t spi_txbuff[32+1] ; //SPI transmit buffer (payload max 32 bytes + 1 byte for the command)

/****************************************************************************/
  // Minimum ideal SPI bus speed is 2x data rate
  // If we assume 2Mbs data rate and 16Mhz clock, a
  // divider of 4 is the minimum we want.
  // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz

static const char nrf24_datarate_e_str_0[] = "1MBPS";
static const char nrf24_datarate_e_str_1[] = "2MBPS";
static const char nrf24_datarate_e_str_2[] = "250KBPS";
static const char * const nrf24_datarate_e_str_P[] = {
  nrf24_datarate_e_str_0, 
  nrf24_datarate_e_str_1, 
  nrf24_datarate_e_str_2, 
};
static const char nrf24_model_e_str_0[] = "nRF24L01";
static const char nrf24_model_e_str_1[] = "nRF24L01+";
static const char * const nrf24_model_e_str_P[] = {
  nrf24_model_e_str_0, 
  nrf24_model_e_str_1, 
};
static const char nrf24_crclength_e_str_0[] = "Disabled";
static const char nrf24_crclength_e_str_1[] = "8 bits";
static const char nrf24_crclength_e_str_2[] = "16 bits";
static const char * const nrf24_crclength_e_str_P[] = {
  nrf24_crclength_e_str_0, 
  nrf24_crclength_e_str_1, 
  nrf24_crclength_e_str_2, 
};
static const char nrf24_pa_dbm_e_str_0[] = "PA_MIN";
static const char nrf24_pa_dbm_e_str_1[] = "PA_GPIO_LOW";
static const char nrf24_pa_dbm_e_str_2[] = "PA_GPIO_HIGH";
static const char nrf24_pa_dbm_e_str_3[] = "PA_MAX";
static const char * const nrf24_pa_dbm_e_str_P[] = { 
  nrf24_pa_dbm_e_str_0, 
  nrf24_pa_dbm_e_str_1, 
  nrf24_pa_dbm_e_str_2, 
  nrf24_pa_dbm_e_str_3, 
};

static const char nrf24_csn_e_str_0[] = "CE0 (PI Hardware Driven)";
static const char nrf24_csn_e_str_1[] = "CE1 (PI Hardware Driven)";
static const char nrf24_csn_e_str_2[] = "CE2 (PI Hardware Driven)";
static const char nrf24_csn_e_str_3[] = "Custom GPIO Software Driven";
static const char * const nrf24_csn_e_str_P[] = {
  nrf24_csn_e_str_0,
  nrf24_csn_e_str_1,
  nrf24_csn_e_str_2,
  nrf24_csn_e_str_3,
};


static const uint8_t child_pipe[] =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const uint8_t child_pipe_enable[] =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void nrf24_init(uint8_t _cepin, uint8_t _cspin, uint32_t _spi_speed) {
    ce_pin = _cepin;
    csn_pin = _cspin;
    spi_speed = _spi_speed;
    p_variant = FALSE;
    payload_size = 32; 
    dynamic_payloads_enabled = FALSE;
    addr_width = 5; //,pipe0_reading_address(0) 
}
/************************/
/* REGISTER FUNCTION    */
/************************/

uint8_t read_register_bytes(uint8_t reg, uint8_t* buf, uint8_t len) 
{
  uint8_t status;
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;
  uint8_t size = len + 1; // Add register value to transmit buffer

  *ptx++ = ( R_REGISTER | ( REGISTER_MASK & reg ) );
  while (len--){
    *ptx++ = NOP ; // Dummy operation, just for reading
  }
  bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);

  status = *prx++; // status is 1st byte of receive buffer

  // decrement before to skip status byte
  while ( --size ){
    *buf++ = *prx++;
  }

  return status;
}

uint8_t read_register(uint8_t reg) 
{
  
  uint8_t result;
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;
  
  *ptx++ = ( R_REGISTER | ( REGISTER_MASK & reg ) );
  *ptx++ = NOP ; // Dummy operation, just for reading

  bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);

  result = *++prx;   // result is 2nd byte of receive buffer

  return result;  
}

uint8_t write_register_bytes(uint8_t reg, const uint8_t* buf, uint8_t len) 
{
  uint8_t status;
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;
  uint8_t size = len + 1; // Add register value to transmit buffer

  *ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- ) {
    *ptx++ = *buf++;
  }
  bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);

  status = *prx; // status is 1st byte of receive buffer
  return status;
}

uint8_t write_register(uint8_t reg, uint8_t value) {
  uint8_t status;
  
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;

  *ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
  *ptx = value ;

    bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);  
   // printf("bcm2835_spi_transfernb\n", ); 
  status = *prx++; // status is 1st byte of receive buffer
  return status;
}

// uint8_t write_payload (const void *buf, uint8_t len) {
//   uint8_t status;
//   uint8_t * prx = spi_rxbuff;
//   uint8_t * ptx = spi_txbuff;
//   uint8_t size ;

//   const uint8_t* current = (const uint8_t*)(buf);

//   uint8_t data_len = min(len,payload_size);
//   uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

//   size = data_len + blank_len + 1 ; // Add register value to transmit buffer

//   if (debug)
//     printf("[Writing %u bytes %u blanks]",data_len,blank_len);

//   *ptx++ =  W_TX_PAYLOAD;
//   while ( data_len-- )
//     *ptx++ =  *current++;
//   while ( blank_len-- )
//     *ptx++ =  0;

//   bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);

//   status = *prx; // status is 1st byte of receive buffer
//   return status;
// }
uint8_t write_payload(const void* buf, uint8_t len, const uint8_t writeType)
{
  uint8_t status;
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;
  uint8_t size ;

  const uint8_t* current = (const uint8_t*)(buf);

  uint8_t data_len = min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

  size = data_len + blank_len + 1 ; // Add register value to transmit buffer

  if (debug)
    printf("[Writing %u bytes %u blanks]",data_len,blank_len);

  *ptx++ =  W_TX_PAYLOAD;
  while ( data_len-- )
    *ptx++ =  *current++;
  while ( blank_len-- )
    *ptx++ =  0;

  bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);

  status = *prx; // status is 1st byte of receive buffer


  return status;
}

uint8_t read_payload (void *buf, uint8_t len) {
  uint8_t status;
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;
  uint8_t size ;

  uint8_t* current = (uint8_t*)(buf);

  uint8_t data_len = min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

  size = data_len + blank_len + 1; // Add register value to transmit buffer

  if (debug)
    printf("[Reading %u bytes %u blanks]",data_len,blank_len);

  *ptx++ =  R_RX_PAYLOAD;
  while(size--)
    *ptx++ = NOP;

  // Size has been lost during while, re affect
  size = data_len + blank_len + 1; // Add register value to transmit buffer

  bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);

  // 1st byte is status
  status = *prx++;

  // Decrement before to skip 1st status byte
  while ( --size )
    *current++ = *prx++;

  return status;
}

uint8_t nrf24_flush_rx (void) {
  uint8_t status;
  status = bcm2835_spi_transfer( FLUSH_RX );
  return status;
}

uint8_t nrf24_flush_tx (void) {
  uint8_t status;
  status = bcm2835_spi_transfer( FLUSH_TX );
  return status;
}

uint8_t get_status (void) {
  return bcm2835_spi_transfer( NOP );
}

void print_status (uint8_t status) {
  printf("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n",
           status,
           (status & _BV(RX_DR))?1:0,
           (status & _BV(TX_DS))?1:0,
           (status & _BV(MAX_RT))?1:0,
           ((status >> RX_P_NO) & 0b111),
           (status & _BV(TX_FULL))?1:0
          );
}

void print_observe_tx (uint8_t value) {
  printf("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n",
           value,
           (value >> PLOS_CNT) & 0b1111,
           (value >> ARC_CNT) & 0b1111
          );
}

void print_byte_register (const char *name, uint8_t reg, uint8_t qty) {
  char extra_tab = strlen(name) < 8 ? '\t' : 0;
  printf("%s\t%c =", name, extra_tab);
  while (qty--)
    printf(" 0x%02x",read_register(reg++));
  printf("\n");
}

void print_address_register (const char *name, uint8_t reg, uint8_t qty) {
  char extra_tab = strlen(name) < 8 ? '\t' : 0;
  printf("%s\t%c =",name,extra_tab);

  while (qty--)
  {
    uint8_t buffer[addr_width];
    read_register_bytes(reg++,buffer,sizeof buffer);

    printf(" 0x");
    uint8_t* bufptr = buffer + sizeof buffer;
    while( --bufptr >= buffer )
      printf("%02x",*bufptr);
  }

  printf("\r\n");
}

void nrf24_setChannel(uint8_t channel)
{
  const uint8_t max_channel = 127;
  write_register(RF_CH,min(channel,max_channel));
}

uint8_t nrf24_getPayloadSize(void)
{
  return payload_size;
}

// Display NRF24L01 details
void nrf24_printDetails(void)
{
  printf("================ SPI Configuration ================\n" );

  if (csn_pin < BCM2835_SPI_CS_NONE )
  {
    printf("CSN Pin  \t = %s\n",nrf24_csn_e_str_P[csn_pin]);
  }
  else
  {
    printf("CSN Pin  \t = Custom GPIO%d%s\n", csn_pin,
      csn_pin==RPI_V2_GPIO_P1_26 ? " (CE1) Software Driven" : "" );
  }

  printf("CE Pin  \t = Custom GPIO%d\n", ce_pin );

  // SPI Bus Speed
  printf("Clock Speed\t = " );
  switch (spi_speed)
  {
    case BCM2835_SPI_SPEED_64MHZ : printf("64 Mhz");  break ;
    case BCM2835_SPI_SPEED_32MHZ : printf("32 Mhz");  break ;
    case BCM2835_SPI_SPEED_16MHZ : printf("16 Mhz");  break ;
    case BCM2835_SPI_SPEED_8MHZ  : printf("8 Mhz"); break ;
    case BCM2835_SPI_SPEED_4MHZ  : printf("4 Mhz"); break ;
    case BCM2835_SPI_SPEED_2MHZ  : printf("2 Mhz"); break ;
    case BCM2835_SPI_SPEED_1MHZ  : printf("1 Mhz"); break ;
    case BCM2835_SPI_SPEED_512KHZ: printf("512 KHz"); break ;
    case BCM2835_SPI_SPEED_256KHZ: printf("256 KHz"); break ;
    case BCM2835_SPI_SPEED_128KHZ: printf("128 KHz"); break ;
    case BCM2835_SPI_SPEED_64KHZ : printf("64 KHz");  break ;
    case BCM2835_SPI_SPEED_32KHZ : printf("32 KHz");  break ;
    case BCM2835_SPI_SPEED_16KHZ : printf("16 KHz");  break ;
    case BCM2835_SPI_SPEED_8KHZ  : printf("8 KHz"); break ;
    default : printf("Probably Bad !!!"); break ;
  }
  printf("\n");

  printf("================ NRF Configuration ================\n" );

  print_status(get_status());

  print_address_register("RX_ADDR_P0-1",RX_ADDR_P0, 2);
  print_byte_register("RX_ADDR_P2-5",RX_ADDR_P2, 4);
  print_address_register("TX_ADDR",TX_ADDR, 1);

  print_byte_register("RX_PW_P0-6",RX_PW_P0,6);
  print_byte_register("EN_AA",EN_AA, 1);
  print_byte_register("EN_RXADDR",EN_RXADDR, 1);
  print_byte_register("RF_CH",RF_CH, 1);
  print_byte_register("RF_SETUP",RF_SETUP, 1);
  print_byte_register("CONFIG",CONFIG, 1);
  print_byte_register("DYNPD/FEATURE",DYNPD,2);
  printf("================ Extra info ================\n" );
  print_byte_register("SETUP_AW",SETUP_AW,1);
  print_byte_register("SETUP_RETR",SETUP_RETR,1);
  print_byte_register("OBSERVE_TX",OBSERVE_TX,1);
  print_byte_register("CD",CD,1);
  print_byte_register("FIFO_STATUS",FIFO_STATUS,1);
  print_byte_register("RX_PW_P0-5",RX_PW_P0,6);

  printf("Data Rate\t = %s\r\n",nrf24_datarate_e_str_P[nrf24_getDataRate()]);
  printf("Model\t\t = %s\r\n",nrf24_model_e_str_P[nrf24_isPVariant()]);
  printf("CRC Length\t = %s\r\n",nrf24_crclength_e_str_P[nrf24_getCRCLength()]);
  printf("PA Power\t = %s\r\n",nrf24_pa_dbm_e_str_P[nrf24_getPALevel()]);
}

bool nrf24_begin(void) {
    debug = FALSE;
  #if defined(DEBUG)
    debug = TRUE;
    #endif
  // This initialize the SPI bus with
  // csn pin as chip select (custom or not)


  // Init BCM2835 chipset for talking with us
  if (!bcm2835_init()){
    return FALSE;
  }

  switch(csn_pin){     //Ensure valid hardware CS pin
    case 0: break;
    case 1: break;
    case 8: csn_pin = 0; break;
    case 7: csn_pin = 1; break;
    default: csn_pin = 0; break;
  }


  bcm2835_spi_begin();
  // used to drive custom I/O to trigger my logic analyser
  // bcm2835_gpio_fsel(GPIO_CTRL_PIN , BCM2835_GPIO_FSEL_OUTP);

  // start the SPI library:
  // Note the NRF24 wants mode 0, MSB first and default to 1 Mbps
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
  // Set SPI bus Speed
  bcm2835_spi_setClockDivider(spi_speed);
    // Choose hardware CSN pin
  bcm2835_spi_chipSelect(csn_pin);

  // Initialise the CE pin of NRF24 (chip enable) after the CSN pin, so that
  // The input mode is not changed if using one of the hardware CE pins
  bcm2835_gpio_fsel(ce_pin, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(ce_pin, LOW);
  // wait 100ms
  delay(100);
  // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
  // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
  // sizes must never be used. See documentation for a more complete explanation.
  //printf("write_register(%02X, %02X)\n", SETUP_RETR, (0b0100 << ARD) | (0b1111 << ARC));
  nrf24_setRetries(5,15);
  // Determine if this is a p or non-p RF24 module and then
  // reset our data rate back to default value. This works
  // because a non-P variant won't allow the data rate to
  // be set to 250Kbps.
  if( nrf24_setDataRate( RF24_250KBPS ) )
  {
    p_variant = TRUE ;
  }
  // Then set the data rate to the slowest (and most reliable) speed supported by all
  // hardware.
  nrf24_setDataRate( RF24_1MBPS ) ;
  // Initialize CRC and request 2-byte (16bit) CRC
  nrf24_setCRCLength( RF24_CRC_16 ) ;
  nrf24_toggle_features();
  write_register(FEATURE,0 );
  write_register(DYNPD,0);
  // Reset current status
  // Notice reset and flush is the last thing we do
  write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Set up default configuration.  Callers can always change it later.
  // This channel should be universally safe and not bleed over into adjacent
  // spectrum.
  nrf24_setChannel(76);

  // Flush buffers
  //flush_rx();
  nrf24_flush_tx();

  nrf24_powerUp();

  // Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
  // PTX should use only 22uA of power
  write_register(CONFIG, ( read_register(CONFIG) ) & ~_BV(PRIM_RX) );

  return TRUE;
}

void nrf24_startListening(void)
{
  nrf24_powerUp();
  write_register(CONFIG, read_register(CONFIG) | _BV(PRIM_RX));
  write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Restore the pipe0 adddress, if exists
  if (pipe0_reading_address[0] > 0){
    write_register_bytes(RX_ADDR_P0, pipe0_reading_address,addr_width); 
  }
  // Flush buffers
  //flush_rx();
  nrf24_flush_tx();

  // Go!
  bcm2835_gpio_write(ce_pin, HIGH);

  // wait for the radio to come up (130us actually only needed)
  delayMicroseconds(130);
}

void nrf24_stopListening(void)
{
  bcm2835_gpio_write(ce_pin, LOW);
  nrf24_flush_tx();
  nrf24_flush_rx();
  delayMicroseconds(150);
  write_register(CONFIG, ( read_register(CONFIG) ) & ~_BV(PRIM_RX) );  
  delayMicroseconds(150);
}

void nrf24_powerDown(void)
{
  bcm2835_gpio_write(ce_pin, LOW);
  write_register(CONFIG,read_register(CONFIG) & ~_BV(PWR_UP));
}

void nrf24_powerUp(void)
{
  bool up = read_register(CONFIG) & _BV(PWR_UP);

  if(! up ){
  write_register(CONFIG, ( read_register(CONFIG) | _BV(PWR_UP) ));
  delay(5);
  }
}

#if defined (FAILURE_HANDLING)
void nrf24_errNotify(){
  if(debug){ printf("HARDWARE FAIL\n\r"); } 
  failureDetect = TRUE;
}
#endif

bool nrf24_write( const void* buf, uint8_t len, const bool multicast ){

  // Begin the write
  nrf24_startFastWrite(buf,len, multicast);


  //Wait until complete or failed
  #if defined (FAILURE_HANDLING)
    uint32_t timer = millis();
  #endif 
    // If this hangs, it ain't coming back, no sense in timing out
  while( ! ( get_status()  & ( _BV(TX_DS) | _BV(MAX_RT) ))) { 
    #if defined (FAILURE_HANDLING)
      if(millis() - timer > 175){     
        nrf24_errNotify();
        return 0;             
      }
    #endif
  }
  bcm2835_gpio_write(ce_pin, LOW);
  uint8_t status = write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
  

  //Max retries exceeded
  if( status & _BV(MAX_RT)){
    nrf24_flush_tx(); //Only going to be 1 packet int the FIFO at a time using this method, so just flush
    return 0;
  }
  //TX OK 1 or 0
  return 1;
}

bool nrf24_send( const void* buf, uint8_t len ){
  return nrf24_write(buf,len,0);
}


//For general use, the interrupt flags are not important to clear
bool nnrf24_writeBlocking( const void* buf, uint8_t len, uint32_t timeout )
{
  //Block until the FIFO is NOT full.
  //Keep track of the MAX retries and set auto-retry if seeing failures
  //This way the FIFO will fill up and allow blocking until packets go through
  //The radio will auto-clear everything in the FIFO as long as CE remains high

  uint32_t timer = millis();                //Get the time that the payload transmission started

  while( ( get_status()  & ( _BV(TX_FULL) ))) {     //Blocking only if FIFO is full. This will loop and block until TX is successful or timeout

    if( get_status() & _BV(MAX_RT)){            //If MAX Retries have been reached
      nrf24_reUseTX();                      //Set re-transmit and clear the MAX_RT interrupt flag
      if(millis() - timer > timeout){ return 0; }     //If this payload has exceeded the user-defined timeout, exit and return 0
    }
    #if defined (FAILURE_HANDLING)
      if(millis() - timer > (timeout+75) ){     
        nrf24_errNotify();
        return 0;             
      }
    #endif
    }

    //Start Writing
  nrf24_startFastWrite(buf,len,0);                  //Write the payload if a buffer is clear

  return 1;                         //Return 1 to indicate successful transmission
}

void nrf24_reUseTX(){
    write_register(STATUS,_BV(MAX_RT) );        //Clear max retry flag
    //spiTrans( REUSE_TX_PL );
    bcm2835_spi_transfer( REUSE_TX_PL);
    bcm2835_gpio_write(ce_pin, LOW);                      //Re-Transfer packet
    bcm2835_gpio_write(ce_pin, HIGH);
}

/****************************************************************************/

bool nrf24_writeFast( const void* buf, uint8_t len, const bool multicast )
{
  //Block until the FIFO is NOT full.
  //Keep track of the MAX retries and set auto-retry if seeing failures
  //Return 0 so the user can control the retrys and set a timer or failure counter if required
  //The radio will auto-clear everything in the FIFO as long as CE remains high

  #if defined (FAILURE_HANDLING)
    uint32_t timer = millis();
  #endif
  
  while( ( get_status()  & ( _BV(TX_FULL) ))) {       //Blocking only if FIFO is full. This will loop and block until TX is successful or fail

    if( get_status() & _BV(MAX_RT)){
      //reUseTX();                      //Set re-transmit
      write_register(STATUS,_BV(MAX_RT) );        //Clear max retry flag
      return 0;                     //Return 0. The previous payload has been retransmitted
                                //From the user perspective, if you get a 0, just keep trying to send the same payload
    }
    #if defined (FAILURE_HANDLING)
      if(millis() - timer > 75 ){     
        nrf24_errNotify();
        return 0;             
      }
    #endif
    }
         //Start Writing
  nrf24_startFastWrite(buf,len,multicast);

  return 1;
}

bool nrf24_sendFast( const void* buf, uint8_t len ){
  return nrf24_writeFast(buf,len,0);
}

/****************************************************************************/

//Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

void nrf24_startFastWrite( const void* buf, uint8_t len, const bool multicast){ //TMRh20

  //write_payload( buf,len);
  write_payload( buf, len,multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
  bcm2835_gpio_write(ce_pin, HIGH);

}

/****************************************************************************/

void nrf24_startWrite( const void* buf, uint8_t len, const bool multicast )
{

  // Send the payload

  write_payload( buf, len,multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
  bcm2835_gpio_write(ce_pin, HIGH);
  delayMicroseconds(10);
  bcm2835_gpio_write(ce_pin, LOW);
}

/****************************************************************************/

bool nrf24_txStandBy(){
    #if defined (FAILURE_HANDLING)
    uint32_t timer = millis();
  #endif
  while( ! (read_register(FIFO_STATUS) & _BV(TX_EMPTY)) ){
    if( get_status() & _BV(MAX_RT)){
      write_register(STATUS,_BV(MAX_RT) );
      bcm2835_gpio_write(ce_pin, LOW);
      nrf24_flush_tx();    //Non blocking, flush the data
      return 0;
    }
    #if defined (FAILURE_HANDLING)
      if( millis() - timer > 75){
        nrf24_errNotify();
        return 0; 
      }
    #endif
  }

  bcm2835_gpio_write(ce_pin, LOW);         //Set STANDBY-I mode
  return 1;
}

/****************************************************************************/

// bool nrf24_txStandBy(uint32_t timeout){

//   uint32_t start = millis();

//   while( ! (read_register(FIFO_STATUS) & _BV(TX_EMPTY)) ){
//     if( get_status() & _BV(MAX_RT)){
//       write_register(STATUS,_BV(MAX_RT) );
//         bcm2835_gpio_write(ce_pin, LOW);                      //Set re-transmit
//         bcm2835_gpio_write(ce_pin, HIGH);
//         if(millis() - start >= timeout){
//           bcm2835_gpio_write(ce_pin, LOW);
//           flush_tx();
//           return 0;
//         }
//     }
//     #if defined (FAILURE_HANDLING)
//       if( millis() - start > (timeout+75)){
//         errNotify();
//         return 0; 
//       }
//     #endif    
//   }
//   bcm2835_gpio_write(ce_pin, LOW);     //Set STANDBY-I mode
//   return 1;

// }
/****************************************************************************/

void nrf24_maskIRQ(bool tx, bool fail, bool rx){

  write_register(CONFIG, ( read_register(CONFIG) ) | fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR  );
}

/****************************************************************************/

uint8_t nrf24_getDynamicPayloadSize(void)
{
  spi_txbuff[0] = R_RX_PL_WID;
  spi_rxbuff[1] = 0xff;

  bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);

  if(spi_rxbuff[1] > 32) { nrf24_flush_rx(); return 0; }

  return spi_rxbuff[1];
}

/****************************************************************************/

// bool nrf24_available(void)
// {
//   return available(NULL);
// }

/****************************************************************************/

bool nrf24_available(uint8_t* pipe_num)
{
    //Check the FIFO buffer to see if data is waitng to be read
  if (!( read_register(FIFO_STATUS) & _BV(RX_EMPTY) )){


    // If the caller wants the pipe number, include that
    if ( pipe_num ){
    uint8_t status = get_status();
      *pipe_num = ( status >> RX_P_NO ) & 0b111;
    }
    return 1;
  }

  return 0;
}

/****************************************************************************/

void nrf24_read( void* buf, uint8_t len )
{
  // Fetch the payload
  read_payload( buf, len );

  //Clear the two possible interrupt flags with one command
  write_register(STATUS,_BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS) );
}

/****************************************************************************/

// void nrf24_whatHappened(bool& tx_ok,bool& tx_fail,bool& rx_ready)
// {
//   // Read the status & reset the status in one easy call
//   // Or is that such a good idea?
//   uint8_t status = write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

//   // Report to the user what happened
//   tx_ok = status & _BV(TX_DS);
//   tx_fail = status & _BV(MAX_RT);
//   rx_ready = status & _BV(RX_DR);
// }

/****************************************************************************/

// void nrf24_openWritingPipe(uint64_t value)
// {
//   // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
//   // expects it LSB first too, so we're good.

//   write_register(RX_ADDR_P0, <uint8_t*>(&value), addr_width);
//   write_register(TX_ADDR, reinterpret_cast<uint8_t*>(&value), addr_width);


//   write_register(RX_PW_P0,payload_size);
// }

/****************************************************************************/
void nrf24_openWritingPipe(const uint8_t *address)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  write_register_bytes(RX_ADDR_P0,address, addr_width);
  write_register_bytes(TX_ADDR, address, addr_width);

  //const uint8_t max_payload_size = 32;
  //write_register(RX_PW_P0,min(payload_size,max_payload_size));
  write_register(RX_PW_P0,payload_size);
}

/****************************************************************************/

void nrf24_setAddressWidth(uint8_t a_width){

  if(a_width -= 2){
    write_register(SETUP_AW,a_width%4);
    addr_width = (a_width%4) + 2;
  }

}

/****************************************************************************/

void nrf24_openReadingPipe(uint8_t child, const uint8_t *address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0){
    memcpy(pipe0_reading_address,address,addr_width);
  }
  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 ){
      write_register_bytes(pgm_read_byte(&child_pipe[child]), address, addr_width);
    }else{
      write_register_bytes(pgm_read_byte(&child_pipe[child]), address, 1);
  }
    write_register(pgm_read_byte(&child_payload_size[child]),payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    write_register(EN_RXADDR,read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));

  }
}

/****************************************************************************/

void nrf24_toggle_features()
{
  bcm2835_spi_transfer( ACTIVATE );
  bcm2835_spi_transfer( 0x73 );
}

/****************************************************************************/

void nrf24_enableDynamicPayloads(void)
{

    // So enable them and try again
    nrf24_toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_DPL) );

  if (debug)
    printf("FEATURE=%i\r\n",read_register(FEATURE));

  // Enable dynamic payload on all pipes
  //
  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

  dynamic_payloads_enabled = TRUE;
}

/****************************************************************************/

void nrf24_enableAckPayload(void)
{
  //
  // enable ack payload and dynamic payload features
  //


    // So enable them and try again
    nrf24_toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

  if (debug)
    printf("FEATURE=%i\r\n",read_register(FEATURE));

  //
  // Enable dynamic payload on pipes 0 & 1
  //
  dynamic_payloads_enabled = TRUE;
  write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
}

/****************************************************************************/

void nrf24_enableDynamicAck(void){
  //
  // enable dynamic ack features
  //
    nrf24_toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_DYN_ACK) );

  if(debug){printf("FEATURE=%i\r\n",read_register(FEATURE));}


}

/****************************************************************************/

void nrf24_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{

  uint8_t * ptx = spi_txbuff;
  uint8_t size ;

  const uint8_t* current = (const uint8_t*)(buf);

  uint8_t data_len = min(len,payload_size);

  size = data_len + 1 ; // Add register value to transmit buffer

  if (debug){
    printf("[Writing %u bytes]",data_len);
  }
  *ptx++ =  W_ACK_PAYLOAD | ( pipe & 0b111 );
  while ( data_len-- ){
    *ptx++ =  *current++;
  }
  bcm2835_spi_transfern( (char *) spi_txbuff, size);  

}


/****************************************************************************/

bool nrf24_isAckPayloadAvailable(void)
{
  return ! read_register(FIFO_STATUS) & _BV(RX_EMPTY);
}

/****************************************************************************/

bool nrf24_isPVariant(void)
{
  return p_variant ;
}

/****************************************************************************/

void nrf24_setAutoAck(bool enable)
{
  if ( enable )
    write_register(EN_AA, 0b111111);
  else
    write_register(EN_AA, 0);
}

/****************************************************************************/

// void nrf24_setAutoAck( uint8_t pipe, bool enable )
// {
//   if ( pipe <= 6 )
//   {
//     uint8_t en_aa = read_register( EN_AA ) ;
//     if( enable )
//     {
//       en_aa |= _BV(pipe) ;
//     }
//     else
//     {
//       en_aa &= ~_BV(pipe) ;
//     }
//     write_register( EN_AA, en_aa ) ;
//   }
// }

/****************************************************************************/

bool nrf24_testCarrier(void)
{
  return ( read_register(CD) & 1 );
}

/****************************************************************************/

bool nrf24_testRPD(void)
{
  return ( read_register(RPD) & 1 ) ;
}

/****************************************************************************/

void nrf24_setPALevel(uint8_t level)
{
  uint8_t setup = read_register(RF_SETUP) & 0b11111000;

  if(level > 3){              // If invalid level, go to max PA
    level = (RF24_PA_MAX << 1) + 1;   // +1 to support the SI24R1 chip extra bit
  }else{
    level = (level << 1) + 1;     // Else set level as requested
  }


  write_register( RF_SETUP, setup |= level ) ;  // Write it to the chip
}

/****************************************************************************/

uint8_t nrf24_getPALevel(void)
{
  return (read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1 ;
}

/****************************************************************************/

bool nrf24_setDataRate(nrf24_datarate_e speed)
{
  bool result = FALSE;
  uint8_t setup = read_register(RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      setup |= _BV(RF_DR_HIGH);
    }
  }
  write_register(RF_SETUP,setup);

  // Verify our result
  if ( read_register(RF_SETUP) == setup )
  {
    result = TRUE;
  }

  return result;
}

/****************************************************************************/

nrf24_datarate_e nrf24_getDataRate( void )
{
  nrf24_datarate_e result ;
  uint8_t dr = read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}

/****************************************************************************/

void nrf24_setCRCLength(nrf24_crclength_e length)
{
  uint8_t config = read_register(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above.
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(EN_CRC);
  }
  else
  {
    config |= _BV(EN_CRC);
    config |= _BV( CRCO );
  }
  write_register( CONFIG, config ) ;
}

/****************************************************************************/

nrf24_crclength_e nrf24_getCRCLength(void)
{
  nrf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = read_register(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;

  if ( config & _BV(EN_CRC ) )
  {
    if ( config & _BV(CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

/****************************************************************************/

void nrf24_disableCRC( void )
{
  uint8_t disable = read_register(CONFIG) & ~_BV(EN_CRC) ;
  write_register( CONFIG, disable ) ;
}

/****************************************************************************/
void nrf24_setRetries(uint8_t delay, uint8_t count)
{
 printf("start setting retry\n");
 write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}
