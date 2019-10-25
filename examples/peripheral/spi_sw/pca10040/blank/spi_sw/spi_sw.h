#include "stdio.h"
#include "stdint.h"

// SPI Pin
#define CLOCK_PIN							6
#define MOSI_PIN							7
#define MISO_PIN							8
#define CS_PIN								5

#define HIGH									1
#define LOW										0


void init_spi_sw(void);

uint8_t write_spi_sw(uint8_t* data_write, uint32_t data_len);

uint8_t read_spi_sw(uint8_t* data_read, uint32_t data_len);
