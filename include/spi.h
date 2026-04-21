#ifndef SPI_H_
#define SPI_H_

#include "stdint.h"
#include "ee14lib.h"

void spi_init();
void spi1_transmit(uint8_t *data,uint32_t size);
void spi1_receive(uint8_t *data,uint32_t size);

#endif /* SPI_H_ */
