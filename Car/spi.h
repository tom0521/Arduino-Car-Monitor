#ifndef SPI_H
#define SPI_H

#include <inttypes.h>

void spi_init();

uint8_t spi_transmit(uint8_t data);

#endif  // SPI_H