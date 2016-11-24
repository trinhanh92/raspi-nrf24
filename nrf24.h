#ifndef __NRF24_H__
#define __NRF24_H__

#include <stdint.h>

typedef uint8_t bool;
#define TRUE 1
#define FALSE 0

#ifndef ADDR_WIDTH
#define ADDR_WIDTH 5
#endif
/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } nrf24_pa_dbm_e;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS, RF24_ERROR} nrf24_datarate_e;

/**
 * CRC Length.  How big (if any) of a CRC is included.
 *
 * For use with setCRCLength()
 */
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } nrf24_crclength_e;

/**
 * Driver for nRF24L01(+) 2.4GHz Wireless Transceiver
 */

void nrf24_init(uint8_t _cepin, uint8_t _cspin, uint32_t _spi_speed);
  /**
   * Begin operation of the chip
   *
   * Call this in setup(), before calling any other methods.
   */
  bool nrf24_begin(void);

  /**
   * Start listening on the pipes opened for reading.
   *
   * Be sure to call openReadingPipe() first.  Do not call write() while
   * in this mode, without first calling stopListening().  Call
   * isAvailable() to check for incoming traffic, and read() to get it.
   */
  void nrf24_startListening(void);

  /**
   * Stop listening for incoming messages
   *
   * Do this before calling write().
   */
  void nrf24_stopListening(void);

  /**
   * Write to the open writing pipe
   *
   * Be sure to call openWritingPipe() first to set the destination
   * of where to write to.
   *
   * This blocks until the message is successfully acknowledged by
   * the receiver or the timeout/retransmit maxima are reached.  In
   * the current configuration, the max delay here is 60ms.
   *
   * The maximum size of data written is the fixed payload size, see
   * getPayloadSize().  However, you can write less, and the remainder
   * will just be filled with zeroes.
   *
   * @param buf Pointer to the data to be sent
   * @param len Number of bytes to be sent
   * @return True if the payload was delivered successfully false if not
   */
  bool nrf24_send( const void* buf, uint8_t len );

  /**
   * Test whether there are bytes available to be read
   *
   * @return True if there is a payload available, false if none is
   */
  // bool nrf24_available(void);

  /**
   * Read the payload
   *
   * Return the last payload received
   *
   * The size of data read is the fixed payload size, see getPayloadSize()
   *
   * @note I specifically chose 'void*' as a data type to make it easier
   * for beginners to use.  No casting needed.
   *
   * @param buf Pointer to a buffer where the data should be written
   * @param len Maximum number of bytes to read into the buffer
   * @return True if the payload was delivered successfully false if not
   */
  void nrf24_read( void* buf, uint8_t len );

  /**
   * New: Open a pipe for writing
   *
   * Only one pipe can be open at once, but you can change the pipe
   * you'll write to. Call stopListening() first.
   *
   * Addresses are assigned via a byte array, default is 5 byte address length
   *
   * Usage is exactly the same as before, except for declaring the array
   *
   * @code
   *   uint8_t addresses[][6] = {"1Node","2Node"};
   *   openWritingPipe(addresses[0]);
   * @endcode
   * @see setAddressWidth
   *
   * @param address The address of the pipe to open. Coordinate these pipe
   * addresses amongst nodes on the network.
   */

  void nrf24_openWritingPipe(const uint8_t *address);

  /**
   * Open a pipe for reading
   *
   * Up to 6 pipes can be open for reading at once.  Open all the
   * reading pipes, and then call startListening().
   *
   * @see openWritingPipe
   * @see setAddressWidth
   *
   * @warning Pipes 1-5 should share the same address, except the first byte.
   * Only the first byte in the array should be unique, e.g.
   * @code
   *   uint8_t addresses[][6] = {"1Node","2Node"};
   *   openReadingPipe(1,addresses[0]);
   *   openReadingPipe(2,addresses[1]);
   * @endcode
   *
   * @warning Pipe 0 is also used by the writing pipe.  So if you open
   * pipe 0 for reading, and then startListening(), it will overwrite the
   * writing pipe.  Ergo, do an openWritingPipe() again before write().
   *
   * @param number Which pipe# to open, 0-5.
   * @param address The 24, 32 or 40 bit address of the pipe to open.
   */

  void nrf24_openReadingPipe(uint8_t number, const uint8_t *address);

  /**
   * Empty the transmit buffer
   *
   * @return Current value of status register
   */
  uint8_t nrf24_flush_tx(void);

  void nrf24_toggle_features(void);
  /**@}*/
  /**
   * @name Optional Configurators
   *
   *  Methods you can use to get or set the configuration of the chip.
   *  None are required.  Calling begin() sets up a reasonable set of
   *  defaults.
   */
  /**@{*/
  /**
   * Set the number and delay of retries upon failed submit
   *
   * @param delay How long to wait between each retry, in multiples of 250us,
   * max is 15.  0 means 250us, 15 means 4000us.
   * @param count How many retries before giving up, max 15
   */
  void nrf24_setRetries(uint8_t delay, uint8_t count);

  /**
   * Set RF communication channel
   *
   * @param channel Which RF channel to communicate on, 0-127
   */
  void nrf24_setChannel(uint8_t channel);

  /**
   * Set Static Payload Size
   *
   * This implementation uses a pre-stablished fixed payload size for all
   * transmissions.  If this method is never called, the driver will always
   * transmit the maximum payload size (32 bytes), no matter how much
   * was sent to write().
   *
   * @todo Implement variable-sized payloads feature
   *
   * @param size The number of bytes in the payload
   */
  void nrf24_setPayloadSize(uint8_t size);

  /**
   * Get Static Payload Size
   *
   * @see setPayloadSize()
   *
   * @return The number of bytes in the payload
   */
  uint8_t nrf24_getPayloadSize(void);

  /**
   * Get Dynamic Payload Size
   *
   * For dynamic payloads, this pulls the size of the payload off
   * the chip
   *
   * @return Payload length of last-received dynamic payload
   */
  uint8_t nrf24_getDynamicPayloadSize(void);

  /**
   * Enable custom payloads on the acknowledge packets
   *
   * Ack payloads are a handy way to return data back to senders without
   * manually changing the radio modes on both units.
   *
   * @see examples/pingpair_pl/pingpair_pl.pde
   */
  void nrf24_enableAckPayload(void);

  /**
   * Enable dynamically-sized payloads
   *
   * This way you don't always have to send large packets just to send them
   * once in a while.  This enables dynamic payloads on ALL pipes.
   *
   * @see examples/pingpair_pl/pingpair_dyn.pde
   */
  void nrf24_enableDynamicPayloads(void);

   /**
   * Enable dynamic ACKs (single write multicasting) for chosen messages
   *
   * @note To enable full multicasting or per-pipe multicast, use setAutoAck()
   *
   * @warning This MUST be called prior to attempting single write NOACK calls
   * @code
   * radio.enableDynamicAck();
   * radio.write(&data,32,1);  // Sends a payload with no acknowledgement requested
   * radio.write(&data,32,0);  // Sends a payload using auto-retry/autoACK
   * @endcode
   */
  void nrf24_enableDynamicAck(void);

  /**
   * Determine whether the hardware is an nRF24L01+ or not.
   *
   * @return true if the hardware is nRF24L01+ (or compatible) and false
   * if its not.
   */
  bool nrf24_isPVariant(void) ;

  /**
   * Enable or disable auto-acknowlede packets
   *
   * This is enabled by default, so it's only needed if you want to turn
   * it off for some reason.
   *
   * @param enable Whether to enable (true) or disable (false) auto-acks
   */
  void nrf24_setAutoAck(bool enable);

  /**
   * Enable or disable auto-acknowlede packets on a per pipeline basis.
   *
   * AA is enabled by default, so it's only needed if you want to turn
   * it off/on for some reason on a per pipeline basis.
   *
   * @param pipe Which pipeline to modify
   * @param enable Whether to enable (true) or disable (false) auto-acks
   */
  // void nrf24_setAutoAck( uint8_t pipe, bool enable ) ;

  /**
   * Set Power Amplifier (PA) level to one of four levels.
   * Relative mnemonics have been used to allow for future PA level
   * changes. According to 6.5 of the nRF24L01+ specification sheet,
   * they translate to: RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm,
   * RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm.
   *
   * @param level Desired PA level.
   */
  void nrf24_setPALevel( uint8_t level ) ;

  /**
   * Fetches the current PA level.
   *
   * @return Returns a value from the rf24_pa_dbm_e enum describing
   * the current PA setting. Please remember, all values represented
   * by the enum mnemonics are negative dBm. See setPALevel for
   * return value describingiptions.
   */
  uint8_t nrf24_getPALevel( void ) ;

  /**
   * Set the transmission data rate
   *
   * @warning setting RF24_250KBPS will fail for non-plus units
   *
   * @param speed RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
   * @return true if the change was successful
   */
  bool nrf24_setDataRate(nrf24_datarate_e speed);

  /**
   * Fetches the transmission data rate
   *
   * @return Returns the hardware's currently configured datarate. The value
   * is one of 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS, as defined in the
   * rf24_datarate_e enum.
   */
  nrf24_datarate_e nrf24_getDataRate( void ) ;

  /**
   * Set the CRC length
   *
   * @param length RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
   */
  void nrf24_setCRCLength(nrf24_crclength_e length);

  /**
   * Get the CRC length
   *
   * @return RF24_DISABLED if disabled or RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
   */
  nrf24_crclength_e nrf24_getCRCLength(void);

  /**
   * Disable CRC validation
   *
   */
  void nrf24_disableCRC( void ) ;


  
  /**@}*/
  /**
   * @name Deprecated
   *
   *  Methods provided for backwards compabibility.
   */
  /**@{*/

  /**
   * Open a pipe for writing
   *
   * Only one pipe can be open at once, but you can change the pipe
   * you'll listen to.  Do not call this while actively listening.
   * Remember to stopListening() first.
   *
   * Addresses are 40-bit hex values, e.g.:
   *
   * @code
   *   openWritingPipe(0xF0F0F0F0F0);
   * @endcode
   *
   * @param address The 40-bit address of the pipe to open.  This can be
   * any value whatsoever, as long as you are the only one writing to it
   * and only one other radio is listening to it.  Coordinate these pipe
   * addresses amongst nodes on the network.
   */
  // void openWritingPipe(uint64_t address);

  /**
   * Open a pipe for reading
   *
   * Up to 6 pipes can be open for reading at once.  Open all the
   * reading pipes, and then call startListening().
   *
   * @see openWritingPipe
   *
   * @warning Pipes 1-5 should share the first 32 bits.
   * Only the least significant byte should be unique, e.g.
   * @code
   *   openReadingPipe(1,0xF0F0F0F0AA);
   *   openReadingPipe(2,0xF0F0F0F066);
   * @endcode
   *
   * @warning Pipe 0 is also used by the writing pipe.  So if you open
   * pipe 0 for reading, and then startListening(), it will overwrite the
   * writing pipe.  Ergo, do an openWritingPipe() again before write().
   *
   * @todo Enforce the restriction that pipes 1-5 must share the top 32 bits
   *
   * @param number Which pipe# to open, 0-5.
   * @param address The 40-bit address of the pipe to open.
   */
  // void openReadingPipe(uint8_t number, uint64_t address);



  // /**@}*/
  /**
   * @name Advanced Operation
   *
   *  Methods you can use to drive the chip in more advanced ways
   */
  /**@{*/

  /**
   * Print a giant block of debugging information to stdout
   *
   * @warning Does nothing if stdout is not defined.  See fdevopen in stdio.h
   */
  void nrf24_printDetails(void);

  /**
   * Enter low-power mode
   *
   * To return to normal power mode, either write() some data or
   * startListening, or powerUp().
   */
  void nrf24_powerDown(void);

  /**
   * Leave low-power mode - making radio more responsive
   *
   * To return to low power mode, call powerDown().
   */
  void nrf24_powerUp(void) ;

/**
  * Write for single NOACK writes. Disables acknowledgements/autoretries for a single write.
  *
  * @note enableDynamicAck() must be called to enable this feature
  *
  * Can be used with enableAckPayload() to request a response
  * @see enableDynamicAck()
  * @see setAutoAck()
  * @see write()
  *
  * @param buf Pointer to the data to be sent
  * @param len Number of bytes to be sent
  * @param multicast Request ACK (0), NOACK (1)
  */
  bool nrf24_write( const void* buf, uint8_t len, const bool multicast );

  /**
   * @note Optimization: New Command   *
   * This will not block until the 3 FIFO buffers are filled with data.
   * Once the FIFOs are full, writeFast will simply wait for success or
   * timeout, and return 1 or 0 respectively. From a user perspective, just
   * keep trying to send the same data. The library will keep auto retrying
   * the current payload using the built in functionality.
   * @warning It is important to never keep the nRF24L01 in TX mode for more than 4ms at a time. If the auto
   * retransmit is enabled, the nRF24L01 is never in TX mode long enough to disobey this rule. Allow the FIFO
   * to clear by issuing txStandBy() or ensure appropriate time between transmissions.
   *
   * ONLY max retry interrupt flags will be cleared when writeFast is called
   *
   * @code
   * Example (Partial blocking):
   *
   *			radio.writeFast(&buf,32);  // Writes 1 payload to the buffers
   *			txStandBy();     		   // Returns 0 if failed. 1 if success. Blocks only until MAX_RT timeout or success. Data flushed on fail.
   *
   *			radio.writeFast(&buf,32);  // Writes 1 payload to the buffers
   *			txStandBy(1000);		   // Using extended timeouts, returns 1 if success. Retries failed payloads for 1 seconds before returning 0.
   * @endcode
   *
   * @see txStandBy()
   * @see write()
   * @see writeBlocking()
   *
   * @param buf Pointer to the data to be sent
   * @param len Number of bytes to be sent
   * @return True if the payload was delivered successfully false if not
   */
  bool nrf24_sendFast( const void* buf, uint8_t len );

  /**
  * WriteFast for single NOACK writes. Disables acknowledgements/autoretries for a single write.
  *
  * @note enableDynamicAck() must be called to enable this feature
  * @see enableDynamicAck()
  * @see setAutoAck()
  *
  * @param buf Pointer to the data to be sent
  * @param len Number of bytes to be sent
  * @param multicast Request ACK (0) or NOACK (1)
  */
  bool writeFast( const void* buf, uint8_t len, const bool multicast );

  /**
   * @note Optimization: New Command
   * This function extends the auto-retry mechanism to any specified duration.
   * It will not block until the 3 FIFO buffers are filled with data.
   * If so the library will auto retry until a new payload is written
   * or the user specified timeout period is reached.
   * @warning It is important to never keep the nRF24L01 in TX mode for more than 4ms at a time. If the auto
   * retransmit is enabled, the nRF24L01 is never in TX mode long enough to disobey this rule. Allow the FIFO
   * to clear by issuing txStandBy() or ensure appropriate time between transmissions.
   *
   * ONLY max retry interrupt flags will be cleared when writeBlocking is called
   * @code
   * Example (Full blocking):
   *
   *			radio.writeBlocking(&buf,32,1000); //Wait up to 1 second to write 1 payload to the buffers
   *			txStandBy(1000);     			   //Wait up to 1 second for the payload to send. Return 1 if ok, 0 if failed.
   *					  				   		   //Blocks only until user timeout or success. Data flushed on fail.
   * @endcode
   * @note If used from within an interrupt, the interrupt should be disabled until completion, and sei(); called to enable millis().
   * @see txStandBy()
   * @see write()
   * @see writeFast()
   *
   * @param buf Pointer to the data to be sent
   * @param len Number of bytes to be sent
   * @param timeout User defined timeout in milliseconds.
   * @return True if the payload was loaded into the buffer successfully false if not
   */
  bool writeBlocking( const void* buf, uint8_t len, uint32_t timeout );

  /**
   * @note Optimization: New Command
   * This function should be called as soon as transmission is finished to
   * drop the radio back to STANDBY-I mode. If not issued, the radio will
   * remain in STANDBY-II mode which, per the data sheet, is not a recommended
   * operating mode.
   *
   * @note When transmitting data in rapid succession, it is still recommended by
   * the manufacturer to drop the radio out of TX or STANDBY-II mode if there is
   * time enough between sends for the FIFOs to empty.
   *
   * Relies on built-in auto retry functionality.
   *
   * @code
   * Example (Partial blocking):
   *
   *			radio.writeFast(&buf,32);
   *			radio.writeFast(&buf,32);
   *			radio.writeFast(&buf,32);  //Fills the FIFO buffers up
   *			bool ok = txStandBy();     //Returns 0 if failed. 1 if success.
   *					  				   //Blocks only until MAX_RT timeout or success. Data flushed on fail.
   * @endcode
   * @see txStandBy(unsigned long timeout)
   * @return True if transmission is successful
   *
   */
   bool nrf24_txStandBy(void);

  /**
   * @note Optimization: New Command
   *
   * This function allows extended blocking and auto-retries per a user defined timeout
   * @code
   *	Fully Blocking Example:
   *
   *			radio.writeFast(&buf,32);
   *			radio.writeFast(&buf,32);
   *			radio.writeFast(&buf,32);   //Fills the FIFO buffers up
   *			bool ok = txStandBy(1000);  //Returns 0 if failed after 1 second of retries. 1 if success.
   *					  				    //Blocks only until user defined timeout or success. Data flushed on fail.
   * @endcode
   * @note If used from within an interrupt, the interrupt should be disabled until completion, and sei(); called to enable millis().
   * @param timeout Number of milliseconds to retry failed payloads
   * @return True if transmission is successful
   *
   */
   // bool nrf24_txStandBy(uint32_t timeout);

  /**
   * Test whether there are bytes available to be read
   *
   * Use this version to discover on which pipe the message
   * arrived.
   *
   * @param[out] pipe_num Which pipe has the payload available
   * @return True if there is a payload available, false if none is
   */
  bool nrf24_available(uint8_t* pipe_num);

/**
   * Non-blocking write to the open writing pipe used for buffered writes
   *
   * @note Optimization: This function now leaves the CE pin high, so the radio
   * will remain in TX or STANDBY-II Mode until a txStandBy() command is issued.
   * This allows the chip to be used to its full potential in TX mode.
   * @warning It is important to never keep the nRF24L01 in TX mode for more than 4ms at a time. If the auto
   * retransmit is enabled, the nRF24L01 is never in TX mode long enough to disobey this rule. Allow the FIFO
   * to clear by issuing txStandBy() or ensure appropriate time between transmissions.
   *
   * @see write()
   * @see writeFast()
   * @see startWrite()
   * @see writeBlocking()
   *
   * For single noAck writes see:
   * @see enableDynamicAck()
   * @see setAutoAck()
   *
   * @param buf Pointer to the data to be sent
   * @param len Number of bytes to be sent
   * @param multicast Request ACK (0) or NOACK (1)
   * @return True if the payload was delivered successfully false if not
   */
  void nrf24_startFastWrite( const void* buf, uint8_t len, const bool multicast );

  /**
   * Non-blocking write to the open writing pipe
   *
   * Just like write(), but it returns immediately. To find out what happened
   * to the send, catch the IRQ and then call whatHappened().
   *
   * @note Optimization: This function again behaves as it did previously for backwards-compatibility.
   * with user code. The library uses startFastWrite() internally.
   * This is mainly used for single-payload transactions.
   *
   * @see write()
   * @see writeFast()
   * @see startFastWrite()
   * @see whatHappened()
   *
   * For single noAck writes see:
   * @see enableDynamicAck()
   * @see setAutoAck()
   *
   * @param buf Pointer to the data to be sent
   * @param len Number of bytes to be sent
   * @param multicast Request ACK (0) or NOACK (1)
   *
   */
  void nrf24_startWrite( const void* buf, uint8_t len, const bool multicast );

  /**
   * Optimization: New Command
   *
   * This function is mainly used internally to take advantage of the auto payload
   * re-use functionality of the chip, but can be beneficial to users as well.
   *
   * The function will instruct the radio to re-use the data in the FIFO buffers,
   * and instructs the radio to re-send once the timeout limit has been reached.
   * Used by writeFast and writeBlocking to initiate retries when a TX failure
   * occurs. Retries are automatically initiated except with the standard write().
   * This way, data is not flushed from the buffer until switching between modes.
   *
   * @note This is to be used AFTER auto-retry fails if wanting to resend
   * using the built-in payload reuse features.
   * After issuing reUseTX(), it will keep reending the same payload forever or until
   * a payload is written to the FIFO, or a flush_tx command is given.
   */
   void nrf24_reUseTX();

  /**
   * Write an ack payload for the specified pipe
   *
   * The next time a message is received on @p pipe, the data in @p buf will
   * be sent back in the acknowledgement.
   *
   * @warning According to the data sheet, only three of these can be pending
   * at any time.  I have not tested this.
   *
   * @param pipe Which pipe# (typically 1-5) will get this response.
   * @param buf Pointer to data that is sent
   * @param len Length of the data to send, up to 32 bytes max.  Not affected
   * by the static payload set by setPayloadSize().
   */
  void nrf24_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);

  /**
   * Determine if an ack payload was received in the most recent call to
   * write().
   *
   * Call read() to retrieve the ack payload.
   *
   * @warning Calling this function clears the internal flag which indicates
   * a payload is available.  If it returns true, you must read the packet
   * out as the very next interaction with the radio, or the results are
   * undefined.
   *
   * @return True if an ack payload is available.
   */
  bool nrf24_isAckPayloadAvailable(void);

  /**
   * Call this when you get an interrupt to find out why
   *
   * Tells you what caused the interrupt, and clears the state of
   * interrupts.
   *
   * @param[out] tx_ok The send was successful (TX_DS)
   * @param[out] tx_fail The send failed, too many retries (MAX_RT)
   * @param[out] rx_ready There is a message waiting to be read (RX_DS)
   */
  // void nrf24_whatHappened(bool & tx_ok,bool & tx_fail,bool & rx_ready);

  /**
   * Test whether there was a carrier on the line for the
   * previous listening period.
   *
   * Useful to check for interference on the current channel.
   *
   * @return true if was carrier, false if not
   */
  bool nrf24_testCarrier(void);

  /**
   * Test whether a signal (carrier or otherwise) greater than
   * or equal to -64dBm is present on the channel. Valid only
   * on nRF24L01P (+) hardware. On nRF24L01, use testCarrier().
   *
   * Useful to check for interference on the current channel and
   * channel hopping strategies.
   *
   * @return true if signal => -64dBm, false if not
   */
  bool nrf24_testRPD(void) ;

    /**
   * Test whether this is a real radio, or a mock shim for
   * debugging.  Setting either pin to 0xff is the way to
   * indicate that this is not a real radio.
   *
   * @return true if this is a legitimate radio
   */
  // bool nrf24_isValid() { return ce_pin != 0xff && csn_pin != 0xff; }

  /**
  * The radio will generate interrupt signals when a transmission is complete,
  * a transmission fails, or a payload is received. This allows users to mask
  * those interrupts to prevent them from generating a signal on the interrupt
  * pin.
  *
  * @code
  * 	Mask all interrupts except the receive interrupt:
  *
  *		radio.maskIRQ(1,1,0);
  * @endcode
  *
  * @param tx_ok  Mask transmission complete interrupts
  * @param tx_fail  Mask transmit failure interrupts
  * @param rx_ready Mask payload received interrupts
  */
  void nrf24_maskIRQ(bool tx_ok,bool tx_fail,bool rx_ready);

  /**
  * Set the address width from 3 to 5 bytes (24, 32 or 40 bit)
  *
  * @param a_width The address width to use: 3,4 or 5
  */

  void nrf24_setAddressWidth(uint8_t a_width);
#endif
