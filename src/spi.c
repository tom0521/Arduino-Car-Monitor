#include <avr/io.h>
#include "pins.h"
#include "register.h"
#include "spi.h"

/*
 * SPI Initialization
 * 
 * Sets all input and output pins and
 * all flags in the SPI registers
 * 
 */
void spi_init() {
  // Set MOSI and SCK as outputs
  SET_OUTPUT(SPI_MOSI);
  SET_OUTPUT(SPI_SCK);

  // Set MISO as input
  SET_INPUT(SPI_MISO);

  // Enable SPI, Master, and set clock rate fck/16
  // via the SPR1 and SPR0 flags
  SPCR = (1 << SPE) | (1 << MSTR) | (0 << SPR1) | (1 << SPR0);
  SPSR = 0;
}

/*
 * SPI transmit function
 * 
 * Will send the given byte of data over
 * the Serial Parallel Interface and return
 * the byte received, if any
 * 
 */
uint8_t spi_transmit(uint8_t data) {
  // Start transmitting data to slave
  // by storing it in the SPI Data Register
  SPDR = data;

  // Wait for transmission to complete
  // which is signified by the SPIF flag
  // in the SPSR register
  while (!(SPSR & (1 << SPIF)))
    ;

  return SPDR;
}
