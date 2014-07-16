/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "nRF24L01.h"
#include "RF24_config.h"
#include "RF24.h"

/*
 * Local variables for RF24 functionality
 */

static uint8_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role, unused on rpi */
static string spidevice;
static uint32_t spispeed;
static uint8_t csn_pin; /**< SPI Chip select */
static bool wide_band; /* 2Mbs data rate in use? */
static bool p_variant; /* False for RF24L01 and true for RF24L01P */
static uint8_t payload_size; /**< Fixed size of payloads */
static bool ack_payload_available; /**< Whether there is an ack payload waiting */
static bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
static uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
static uint64_t pipe0_reading_address; /**< Last address set on pipe 0 for reading. */

static SPI* spi;

/****************************************************************************/

void RF24_csn(int mode)
{
  // Minimum ideal SPI bus speed is 2x data rate
  // If we assume 2Mbs data rate and 16Mhz clock, a
  // divider of 4 is the minimum we want.
  // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
#ifdef ARDUINO
//  spi->setBitOrder(MSBFIRST);
//  spi->setDataMode(SPI_MODE0);
//  spi->setClockDivider(SPI_CLOCK_DIV4);
#endif
  digitalWrite(csn_pin,mode);
}

/****************************************************************************/

void RF24_ce(int level)
{
  digitalWrite(ce_pin,level);
}

/****************************************************************************/

uint8_t RF24_read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
  uint8_t status;

  RF24_csn(LOW);
  status = spi->transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
    *buf++ = spi->transfer(0xff);

  RF24_csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24_read_register(uint8_t reg)
{
  RF24_csn(LOW);
  spi->transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  uint8_t result = spi->transfer(0xff);

  RF24_csn(HIGH);
  return result;
}

/****************************************************************************/

uint8_t RF24_write_register(uint8_t reg, const uint8_t* buf, uint8_t len)
{
  uint8_t status;

  RF24_csn(LOW);
  status = spi->transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
    spi->transfer(*buf++);

  RF24_csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24_write_register(uint8_t reg, uint8_t value)
{
  uint8_t status;

  IF_SERIAL_DEBUG(printf_P(PSTR("write_register(%02x,%02x)\r\n"),reg,value));

  RF24_csn(LOW);
  status = spi->transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  spi->transfer(value);
  RF24_csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24_write_payload(const void* buf, uint8_t len)
{
  uint8_t status;

  const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

  uint8_t data_len = min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  
  //printf("[Writing %u bytes %u blanks]",data_len,blank_len);
  
  RF24_csn(LOW);
  status = spi->transfer( W_TX_PAYLOAD );
  while ( data_len-- )
    spi->transfer(*current++);
  while ( blank_len-- )
    spi->transfer(0);
  RF24_csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24_read_payload(void* buf, uint8_t len)
{
  uint8_t status;
  uint8_t* current = reinterpret_cast<uint8_t*>(buf);

  uint8_t data_len = min(len,payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  
  //printf("[Reading %u bytes %u blanks]",data_len,blank_len);
  
  RF24_csn(LOW);
  status = spi->transfer( R_RX_PAYLOAD );
  while ( data_len-- )
    *current++ = spi->transfer(0xff);
  while ( blank_len-- )
    spi->transfer(0xff);
  RF24_csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24_flush_rx(void)
{
  uint8_t status;

  RF24_csn(LOW);
  status = spi->transfer( FLUSH_RX );
  RF24_csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24_flush_tx(void)
{
  uint8_t status;

  RF24_csn(LOW);
  status = spi->transfer( FLUSH_TX );
  RF24_csn(HIGH);

  return status;
}

/****************************************************************************/

uint8_t RF24_get_status(void)
{
  uint8_t status;

  RF24_csn(LOW);
  status = spi->transfer( NOP );
  RF24_csn(HIGH);

  return status;
}

/****************************************************************************/

void RF24_print_status(uint8_t status)
{
  printf_P(PSTR("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n"),
           status,
           (status & _BV(RX_DR))?1:0,
           (status & _BV(TX_DS))?1:0,
           (status & _BV(MAX_RT))?1:0,
           ((status >> RX_P_NO) & 0b111),
           (status & _BV(TX_FULL))?1:0
          );
}

/****************************************************************************/

void RF24_print_observe_tx(uint8_t value)
{
  printf_P(PSTR("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n"),
           value,
           (value >> PLOS_CNT) & 0b1111,
           (value >> ARC_CNT) & 0b1111
          );
}

/****************************************************************************/

void RF24_print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
  char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
  printf_P(PSTR(PRIPSTR"\t%c ="),name,extra_tab);
  while (qty--)
    printf_P(PSTR(" 0x%02x"),RF24_read_register(reg++));
  printf_P(PSTR("\r\n"));
}

/****************************************************************************/

void RF24_print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
  char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
  printf_P(PSTR(PRIPSTR"\t%c ="),name,extra_tab);

  while (qty--)
  {
    uint8_t buffer[5];
    RF24_read_register(reg++,buffer,sizeof buffer);

    printf_P(PSTR(" 0x"));
    uint8_t* bufptr = buffer + sizeof buffer;
    while( --bufptr >= buffer )
      printf_P(PSTR("%02x"),*bufptr);
  }

  printf_P(PSTR("\r\n"));
}

/****************************************************************************/

RF24_RF24(string _spidevice, uint32_t _spispeed, uint8_t _cepin)
{
	 strcpy(_spidevice, spidevice);
	 spispeed = _spispeed;
	 ce_pin = _cepin;
	 wide_band = true;
	 p_variant = false;
	 payload_size = 32;
	 ack_payload_available = false;
	 dynamic_payloads_enabled = false;
	 pipe0_reading_address = 0;
}

/****************************************************************************/

RF24_RF24(uint8_t _cepin, uint8_t _cspin):
{
	  ce_pin =_cepin;
	  csn_pin =_cspin;
	  wide_band = true;
	  p_variant = false;
	  payload_size = 32;
	  ack_payload_available = false;
	  dynamic_payloads_enabled = false;
	  pipe0_reading_address = 0;
}

/****************************************************************************/

void RF24_setChannel(uint8_t channel)
{
  // TODO: This method could take advantage of the 'wide_band' calculation
  // done in setChannel() to require certain channel spacing.

  const uint8_t max_channel = 127;
  RF24_write_register(RF_CH,min(channel,max_channel));
}

/****************************************************************************/

void RF24_setPayloadSize(uint8_t size)
{
  const uint8_t max_payload_size = 32;
  payload_size = min(size,max_payload_size);
}

/****************************************************************************/

uint8_t RF24_getPayloadSize(void)
{
  return payload_size;
}

/****************************************************************************/

static const char rf24_datarate_e_str_0[] PROGMEM = "1MBPS";
static const char rf24_datarate_e_str_1[] PROGMEM = "2MBPS";
static const char rf24_datarate_e_str_2[] PROGMEM = "250KBPS";
static const char * const rf24_datarate_e_str_P[] PROGMEM = {
  rf24_datarate_e_str_0,
  rf24_datarate_e_str_1,
  rf24_datarate_e_str_2,
};
static const char rf24_model_e_str_0[] PROGMEM = "nRF24L01";
static const char rf24_model_e_str_1[] PROGMEM = "nRF24L01+";
static const char * const rf24_model_e_str_P[] PROGMEM = {
  rf24_model_e_str_0,
  rf24_model_e_str_1,
};
static const char rf24_crclength_e_str_0[] PROGMEM = "Disabled";
static const char rf24_crclength_e_str_1[] PROGMEM = "8 bits";
static const char rf24_crclength_e_str_2[] PROGMEM = "16 bits" ;
static const char * const rf24_crclength_e_str_P[] PROGMEM = {
  rf24_crclength_e_str_0,
  rf24_crclength_e_str_1,
  rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[] PROGMEM = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] PROGMEM = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] PROGMEM = "PA_HIGH";
static const char rf24_pa_dbm_e_str_3[] PROGMEM = "PA_MAX";
static const char * const rf24_pa_dbm_e_str_P[] PROGMEM = { 
  rf24_pa_dbm_e_str_0,
  rf24_pa_dbm_e_str_1,
  rf24_pa_dbm_e_str_2,
  rf24_pa_dbm_e_str_3,
};

void RF24_printDetails(void)
{

  printf_P(PSTR("SPI device\t = %s\r\n"),spidevice.c_str() );
  printf_P(PSTR("SPI speed\t = %d\r\n"),spispeed);
  printf_P(PSTR("CE GPIO\t = %d\r\n"),ce_pin);
	
  print_status(get_status());

  print_address_register(PSTR("RX_ADDR_P0-1"),RX_ADDR_P0,2);
  print_byte_register(PSTR("RX_ADDR_P2-5"),RX_ADDR_P2,4);
  print_address_register(PSTR("TX_ADDR"),TX_ADDR);

  print_byte_register(PSTR("RX_PW_P0-6"),RX_PW_P0,6);
  print_byte_register(PSTR("EN_AA"),EN_AA);
  print_byte_register(PSTR("EN_RXADDR"),EN_RXADDR);
  print_byte_register(PSTR("RF_CH"),RF_CH);
  print_byte_register(PSTR("RF_SETUP"),RF_SETUP);
  print_byte_register(PSTR("CONFIG"),CONFIG);
  print_byte_register(PSTR("DYNPD/FEATURE"),DYNPD,2);

  printf_P(PSTR("Data Rate\t = %s\r\n"),pgm_read_word(&rf24_datarate_e_str_P[getDataRate()]));
  printf_P(PSTR("Model\t\t = %s\r\n"),pgm_read_word(&rf24_model_e_str_P[isPVariant()]));
  printf_P(PSTR("CRC Length\t = %s\r\n"),pgm_read_word(&rf24_crclength_e_str_P[getCRCLength()]));
  printf_P(PSTR("PA Power\t = %s\r\n"),pgm_read_word(&rf24_pa_dbm_e_str_P[getPALevel()]));
}

/****************************************************************************/

void RF24_begin(int wiringPiMode)
{
  // Initialize pins
  pinMode(ce_pin,OUTPUT);

  if (wiringPiMode == WPI_MODE_PINS) {
    csn_pin = (spidevice == "/dev/spidev0.1") ? 11 : 10;
  } else if ((wiringPiMode == WPI_MODE_GPIO) || (wiringPiMode == WPI_MODE_GPIO_SYS)) {
    csn_pin = (spidevice == "/dev/spidev0.1") ? 7 : 8;
  } else if (wiringPiMode == WPI_MODE_PHYS) {
    csn_pin = (spidevice == "/dev/spidev0.1") ? 26 : 24;
  } else {
    printf("Error, unrecognized WiringPi pin mapping.");
    exit(1);
  }

  pinMode(csn_pin,OUTPUT);

  // Initialize SPI bus
  //spi->begin();
  spi = malloc(sizeof(SPI));

  spi->setdevice(spidevice);
  spi->setspeed(spispeed);
  spi->setbits(8);
  spi->init();

  RF24_ce(LOW);
  RF24_csn(HIGH);

  // Must allow the radio time to settle else configuration bits will not necessarily stick.
  // This is actually only required following power up but some settling time also appears to
  // be required after resets too. For full coverage, we'll always assume the worst.
  // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
  // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
  // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
  delay( 5 ) ;

  // Adjustments as per gcopeland fork  
  //resetcfg();

  // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
  // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
  // sizes must never be used. See documentation for a more complete explanation.
  RF24_write_register(SETUP_RETR,(0b0101 << ARD) | (0b1111 << ARC));

  // Restore our default PA level
  setPALevel( RF24_PA_MAX ) ;

  // Determine if this is a p or non-p RF24 module and then
  // reset our data rate back to default value. This works
  // because a non-P variant won't allow the data rate to
  // be set to 250Kbps.
  if( setDataRate( RF24_250KBPS ) )
  {
    p_variant = true ;
  }
  
  // Then set the data rate to the slowest (and most reliable) speed supported by all
  // hardware.
  setDataRate( RF24_1MBPS ) ;

  // Initialize CRC and request 2-byte (16bit) CRC
  setCRCLength( RF24_CRC_16 ) ;
  
  // Disable dynamic payloads, to match dynamic_payloads_enabled setting
  RF24_write_register(DYNPD,0);

  // Reset current status
  // Notice reset and flush is the last thing we do
  RF24_write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Set up default configuration.  Callers can always change it later.
  // This channel should be universally safe and not bleed over into adjacent
  // spectrum.
  setChannel(76);

  // Flush buffers
  flush_rx();
  flush_tx();
}

/****************************************************************************/


void RF24_resetcfg(void){
	RF24_write_register(0x00,0x0f);
}

void RF24_startListening(void)
{
  RF24_write_register(CONFIG, RF24_read_register(CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
  RF24_write_register(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Restore the pipe0 adddress, if exists
  if (pipe0_reading_address)
    RF24_write_register(RX_ADDR_P0, reinterpret_cast<const uint8_t*>(&pipe0_reading_address), 5);


  // Adjustments as per gcopeland fork  
  // Flush buffers
  //flush_rx();
  //flush_tx();

  // Go!
  RF24_ce(HIGH);

  // wait for the radio to come up (130us actually only needed)
  delayMicroseconds(130);
}

/****************************************************************************/

void RF24_stopListening(void)
{
  RF24_ce(LOW);
  flush_tx();
  flush_rx();
}

/****************************************************************************/

void RF24_powerDown(void)
{
  RF24_write_register(CONFIG,RF24_read_register(CONFIG) & ~_BV(PWR_UP));

// Adjustments as per gcopeland fork  
  delayMicroseconds(150);
}

/****************************************************************************/

void RF24_powerUp(void)
{
  RF24_write_register(CONFIG,RF24_read_register(CONFIG) | _BV(PWR_UP));
// Adjustments as per gcopeland fork  
  delayMicroseconds(150);
}

/******************************************************************/

bool RF24_write( const void* buf, uint8_t len )
{
  bool result = false;

  // Begin the write
  startWrite(buf,len);

  uint8_t observe_tx;
  uint8_t status;
  uint32_t sent_at = millis();
  const uint32_t timeout = 500; //ms to wait for timeout
  do
  {
    status = RF24_read_register(OBSERVE_TX,&observe_tx,1);
    IF_SERIAL_DEBUG(printf(observe_tx,HEX));
  }
  while( ! ( status & ( _BV(TX_DS) | _BV(MAX_RT) ) ) && ( millis() - sent_at < timeout ) );

  bool tx_ok, tx_fail;
  whatHappened(tx_ok,tx_fail,ack_payload_available);
  
  //printf("%u%u%u\r\n",tx_ok,tx_fail,ack_payload_available);

  result = tx_ok;
  IF_SERIAL_DEBUG(printf(result?"...OK.":"...Failed"));

  // Handle the ack packet
  if ( ack_payload_available )
  {
    ack_payload_length = getDynamicPayloadSize();
    IF_SERIAL_DEBUG(printf("[AckPacket]/"));
    IF_SERIAL_DEBUG(printfln(ack_payload_length,DEC));
  }


  // Disable powerDown and flush_tx as per gcopeland fork
  //powerDown();
  //flush_tx();

  return result;
}
/****************************************************************************/

void RF24_startWrite( const void* buf, uint8_t len )
{
  // Transmitter power-up
  RF24_write_register(CONFIG, ( RF24_read_register(CONFIG) | _BV(PWR_UP) ) & ~_BV(PRIM_RX) );
// Adjustments as per gcopeland fork  
// delayMicroseconds(150);

  // Send the payload
  write_payload( buf, len );

  // Allons!
  RF24_ce(HIGH);
  delayMicroseconds(10);
  RF24_ce(LOW);
}

/****************************************************************************/

uint8_t RF24_getDynamicPayloadSize(void)
{
  uint8_t result = 0;

  RF24_csn(LOW);
  spi->transfer( R_RX_PL_WID );
  result = spi->transfer(0xff);
  RF24_csn(HIGH);

  return result;
}

/****************************************************************************/

bool RF24_available(void)
{
  return available(NULL);
}

/****************************************************************************/

bool RF24_available(uint8_t* pipe_num)
{
  uint8_t status = get_status();

  // Too noisy, enable if you really want lots o data!!
  //IF_SERIAL_DEBUG(print_status(status));

  bool result = ( status & _BV(RX_DR) );

  if (result)
  {
    // If the caller wants the pipe number, include that
    if ( pipe_num )
      *pipe_num = ( status >> RX_P_NO ) & 0b111;

    // Clear the status bit

    // ??? Should this REALLY be cleared now?  Or wait until we
    // actually READ the payload?

    RF24_write_register(STATUS,_BV(RX_DR) );

    // Handle ack payload receipt
    if ( status & _BV(TX_DS) )
    {
      RF24_write_register(STATUS,_BV(TX_DS));
    }
  }

  return result;
}

/****************************************************************************/

bool RF24_read( void* buf, uint8_t len )
{
  // Fetch the payload
  read_payload( buf, len );

  // was this the last of the data available?
  return RF24_read_register(FIFO_STATUS) & _BV(RX_EMPTY);
}

/****************************************************************************/

void RF24_whatHappened(bool& tx_ok,bool& tx_fail,bool& rx_ready)
{
  // Read the status & reset the status in one easy call
  // Or is that such a good idea?
  uint8_t status = RF24_write_register(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Report to the user what happened
  tx_ok = status & _BV(TX_DS);
  tx_fail = status & _BV(MAX_RT);
  rx_ready = status & _BV(RX_DR);
}

/****************************************************************************/

void RF24_openWritingPipe(uint64_t value)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  RF24_write_register(RX_ADDR_P0, reinterpret_cast<uint8_t*>(&value), 5);
  RF24_write_register(TX_ADDR, reinterpret_cast<uint8_t*>(&value), 5);

  const uint8_t max_payload_size = 32;
  RF24_write_register(RX_PW_P0,min(payload_size,max_payload_size));
}

/****************************************************************************/

static const uint8_t child_pipe[] PROGMEM =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] PROGMEM =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const uint8_t child_pipe_enable[] PROGMEM =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void RF24_openReadingPipe(uint8_t child, uint64_t address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0)
    pipe0_reading_address = address;

  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 )
      RF24_write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 5);
    else
      RF24_write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 1);

    RF24_write_register(pgm_read_byte(&child_payload_size[child]),payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    RF24_write_register(EN_RXADDR,RF24_read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
  }
}

/****************************************************************************/

void RF24_toggle_features(void)
{
  RF24_csn(LOW);
  spi->transfer( ACTIVATE );
  spi->transfer( 0x73 );
  RF24_csn(HIGH);
}

/****************************************************************************/

void RF24_enableDynamicPayloads(void)
{
  // Enable dynamic payload throughout the system
  RF24_write_register(FEATURE,RF24_read_register(FEATURE) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! RF24_read_register(FEATURE) )
  {
    // So enable them and try again
    toggle_features();
    RF24_write_register(FEATURE,RF24_read_register(FEATURE) | _BV(EN_DPL) );
  }

  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",RF24_read_register(FEATURE)));

  // Enable dynamic payload on all pipes
  //
  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  RF24_write_register(DYNPD,RF24_read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

  dynamic_payloads_enabled = true;
}

/****************************************************************************/

void RF24_enableAckPayload(void)
{
  //
  // enable ack payload and dynamic payload features
  //

  RF24_write_register(FEATURE,RF24_read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

  // If it didn't work, the features are not enabled
  if ( ! RF24_read_register(FEATURE) )
  {
    // So enable them and try again
    toggle_features();
    RF24_write_register(FEATURE,RF24_read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
  }

  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",RF24_read_register(FEATURE)));

  //
  // Enable dynamic payload on pipes 0 & 1
  //

  RF24_write_register(DYNPD,RF24_read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
}

/****************************************************************************/

void RF24_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{
  const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

  RF24_csn(LOW);
  spi->transfer( W_ACK_PAYLOAD | ( pipe & 0b111 ) );
  const uint8_t max_payload_size = 32;
  uint8_t data_len = min(len,max_payload_size);
  while ( data_len-- )
    spi->transfer(*current++);

  RF24_csn(HIGH);
}

/****************************************************************************/

bool RF24_isAckPayloadAvailable(void)
{
  bool result = ack_payload_available;
  ack_payload_available = false;
  return result;
}

/****************************************************************************/

bool RF24_isPVariant(void)
{
  return p_variant ;
}

/****************************************************************************/

void RF24_setAutoAck(bool enable)
{
  if ( enable )
    RF24_write_register(EN_AA, 0b111111);
  else
    RF24_write_register(EN_AA, 0);
}

/****************************************************************************/

void RF24_setAutoAck( uint8_t pipe, bool enable )
{
  if ( pipe <= 6 )
  {
    uint8_t en_aa = RF24_read_register( EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    RF24_write_register( EN_AA, en_aa ) ;
  }
}

/****************************************************************************/

bool RF24_testCarrier(void)
{
  return ( RF24_read_register(CD) & 1 );
}

/****************************************************************************/

bool RF24_testRPD(void)
{
  return ( RF24_read_register(RPD) & 1 ) ;
}

/****************************************************************************/

void RF24_setPALevel(rf24_pa_dbm_e level)
{
  uint8_t setup = RF24_read_register(RF_SETUP) ;
  setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( level == RF24_PA_MAX )
  {
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }
  else if ( level == RF24_PA_HIGH )
  {
    setup |= _BV(RF_PWR_HIGH) ;
  }
  else if ( level == RF24_PA_LOW )
  {
    setup |= _BV(RF_PWR_LOW);
  }
  else if ( level == RF24_PA_MIN )
  {
    // nothing
  }
  else if ( level == RF24_PA_ERROR )
  {
    // On error, go to maximum PA
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  }

  RF24_write_register( RF_SETUP, setup ) ;
}

/****************************************************************************/

rf24_pa_dbm_e RF24_getPALevel(void)
{
  rf24_pa_dbm_e result = RF24_PA_ERROR ;
  uint8_t power = RF24_read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  // switch uses RAM (evil!)
  if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
  {
    result = RF24_PA_MAX ;
  }
  else if ( power == _BV(RF_PWR_HIGH) )
  {
    result = RF24_PA_HIGH ;
  }
  else if ( power == _BV(RF_PWR_LOW) )
  {
    result = RF24_PA_LOW ;
  }
  else
  {
    result = RF24_PA_MIN ;
  }

  return result ;
}

/****************************************************************************/

bool RF24_setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = RF24_read_register(RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  wide_band = false ;
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    wide_band = false ;
    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      wide_band = true ;
      setup |= _BV(RF_DR_HIGH);
    }
    else
    {
      // 1Mbs
      wide_band = false ;
    }
  }
  RF24_write_register(RF_SETUP,setup);

  // Verify our result
  if ( RF24_read_register(RF_SETUP) == setup )
  {
    result = true;
  }
  else
  {
    wide_band = false;
  }

  return result;
}

/****************************************************************************/

rf24_datarate_e RF24_getDataRate( void )
{
  rf24_datarate_e result ;
  uint8_t dr = RF24_read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
  
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

void RF24_setCRCLength(rf24_crclength_e length)
{
  uint8_t config = RF24_read_register(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;
  
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
  RF24_write_register( CONFIG, config ) ;
}

/****************************************************************************/

rf24_crclength_e RF24_getCRCLength(void)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = RF24_read_register(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;

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

void RF24_disableCRC( void )
{
  uint8_t disable = RF24_read_register(CONFIG) & ~_BV(EN_CRC) ;
  RF24_write_register( CONFIG, disable ) ;
}

/****************************************************************************/
void RF24_setRetries(uint8_t delay, uint8_t count)
{
 RF24_write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}

// vim:ai:cin:sts=2 sw=2 ft=cpp

