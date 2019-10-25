#include "spi_sw.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"

void delay_clock_spi(void)
{
	nrf_delay_us(1);
}

void init_spi_sw(void)
{
	nrf_gpio_pin_clear(CLOCK_PIN);
	nrf_gpio_cfg_output(CLOCK_PIN);	
	nrf_gpio_pin_set(CS_PIN);
	nrf_gpio_cfg_output(CS_PIN);												 
	nrf_gpio_cfg_output(MOSI_PIN); 									 		
	nrf_gpio_cfg_input(MISO_PIN, NRF_GPIO_PIN_NOPULL); 
}

uint8_t write_spi_sw(uint8_t* data_write, uint32_t data_len)
{
	uint8_t bit_buf;
	uint8_t data_temp;
	for(uint32_t i = 0; i < data_len; i++)
		{
			data_temp = data_write[i];
			
			for(uint8_t j = 0; j < 8; j++)
			{
				bit_buf = (data_temp & 0x80)? HIGH : LOW;
				nrf_gpio_pin_write(MOSI_PIN, bit_buf);
				
				nrf_gpio_pin_set(CLOCK_PIN);
				delay_clock_spi();
				nrf_gpio_pin_clear(CLOCK_PIN);
				delay_clock_spi();
				
				data_temp = data_temp << 1;
			}
		}
		return 0;
}

uint8_t read_spi_sw(uint8_t* data_read, uint32_t data_len)
{
		for(uint32_t i = 0; i < data_len; i++)
		{
			data_read[i] = 0;
			
			for(uint8_t j = 0; j < 8; j++)
			{			
				data_read[i] <<= 1;
				
				nrf_gpio_pin_set(CLOCK_PIN);
				delay_clock_spi();
				
				data_read[i] |= nrf_gpio_pin_read(MISO_PIN);
				
				nrf_gpio_pin_clear(CLOCK_PIN);
				delay_clock_spi();
			}
		}

	return 0;
}
