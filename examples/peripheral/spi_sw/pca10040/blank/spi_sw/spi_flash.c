#include "spi_sw.h"
#include "spi_flash.h"
#include "nrf_log.h"

/** Ready Status **/
int16_t status_regis_read_flash(void)
{
	uint8_t cmf[1] = {STATUS_REGIS_READ};
	uint8_t data_read[2];
	
	nrf_gpio_pin_clear(CS_PIN);
	
	write_spi_sw(cmf, sizeof(cmf));
	read_spi_sw(data_read, 2);
	
	nrf_gpio_pin_set(CS_PIN);
	return ((uint16_t) data_read[0] << 8) | data_read[1];
}


/** Write to buffer **/
int8_t write_buf_flash(uint16_t offset, uint8_t* data_wirte, uint32_t data_len)
{
		uint8_t cmf[4] = {WRITE_BUFFER1, DUMMY, (uint8_t)(offset >> 8), (uint8_t)offset};
		
		nrf_gpio_pin_clear(CS_PIN);
		
		write_spi_sw(cmf, sizeof(cmf));
		write_spi_sw(data_wirte, data_len);
		
		nrf_gpio_pin_set(CS_PIN);
		
	return 0;
}

/** Write buffer to page **/
int8_t write_buf_to_page_flash(uint16_t page)
{	
		//uint8_t cmf[4] = {WRITE_BUF_TO_PAGE1, (uint8_t)(page >> 8), (uint8_t)page, DUMMY};
		uint8_t cmf[4] = {WRITE_BUF_TO_PAGE1, (uint8_t)(page >> 7), (uint8_t)(page << 1), DUMMY};
		
		nrf_gpio_pin_clear(CS_PIN);
		
		write_spi_sw(cmf, sizeof(cmf));
		
		nrf_gpio_pin_set(CS_PIN);
		
	return 0;
}

/** Read page to buffer **/
int8_t page_to_buf_flash(uint16_t page) 
{
		uint8_t cmf[4] = {Main_Memory_Page_Read, (uint8_t)(page >> 7), (uint8_t)(page << 1), DUMMY}; //0x53
		
		nrf_gpio_pin_clear(CS_PIN);
		
		write_spi_sw(cmf, sizeof(cmf));
		
		nrf_gpio_pin_set(CS_PIN);
		
	return 0;
}

/** Read in Buffer **/
int8_t read_buf_flash(uint16_t offset, uint8_t* data_read, uint32_t data_len)
{
		uint8_t cmf[4] = {BUF1_READ_LOW, DUMMY, (uint8_t)(offset >> 8), (uint8_t)offset};
		
		nrf_gpio_pin_clear(CS_PIN);
		
		write_spi_sw(cmf, sizeof(cmf));
		
		read_spi_sw(data_read, data_len);
		
		nrf_gpio_pin_set(CS_PIN);
		
	return 0;
}


int8_t write_flash(uint32_t addr, uint8_t* data_wirte, uint32_t data_len)
{
	uint16_t page 					= addr / BYTE_PER_PAGE;
	uint16_t offset 				= addr % BYTE_PER_PAGE;
	uint16_t num_page 			= (data_len - (BYTE_PER_PAGE - offset)) / BYTE_PER_PAGE;
	uint32_t temp_data_len	= 0;
	
	if((addr + data_len) > BYTES_MAX)
	{
		return -1;
	}
	
	if((BYTE_PER_PAGE - offset) >= data_len)
	{
		write_buf_flash(offset, data_wirte, data_len);
		write_buf_to_page_flash(page);
		
		while(1)
		{
			if(status_regis_read_flash() & 0x8000)
			{
				break;	
			}
		}
	}
	else 
	{
		temp_data_len = BYTE_PER_PAGE - offset;
		
		write_buf_flash(offset, data_wirte, temp_data_len);
		write_buf_to_page_flash(page);
		
		while(1)
		{
			if(status_regis_read_flash() & 0x8000)
			{
				break;	
			}
		}
		
		for(uint16_t i = 0; i < num_page; i++)
		{
			page = page + 1;		
			
			write_buf_flash(0, &data_wirte[temp_data_len], BYTE_PER_PAGE);
			write_buf_to_page_flash(page);
			
			temp_data_len = temp_data_len + BYTE_PER_PAGE;
			
			while(1)
			{
				if(status_regis_read_flash() & 0x8000)
				{
					break;	
				}
			}		
		}
		
		if(temp_data_len < data_len)
		{
			page = page + 1;
			
			write_buf_flash(0, &data_wirte[temp_data_len], data_len - temp_data_len);
			write_buf_to_page_flash(page);

			while(1)
			{
				if(status_regis_read_flash() & 0x8000)
				{
					break;	
				}
			}
		}
	}
	
	return 1;
}


int8_t read_flash(uint32_t addr, uint8_t* data_read, uint32_t data_len)
{	
	uint16_t page						= addr / BYTE_PER_PAGE;
	uint16_t offset 				= addr % BYTE_PER_PAGE;	
	uint16_t num_page 			= (data_len - (BYTE_PER_PAGE - offset)) / BYTE_PER_PAGE;
	uint32_t temp_data_len 	= 0;
	
	if((BYTE_PER_PAGE - offset) >= data_len)
	{ 
		page_to_buf_flash(page);
		
		while(1)
		{
			if(status_regis_read_flash() & 0x8000)
			{
				break;	
			}
		}	
			
		read_buf_flash(offset, data_read, data_len);
	}
	else 
	{
		temp_data_len = BYTE_PER_PAGE - offset;
		
		page_to_buf_flash(page);
		
		while(1)
		{
			if(status_regis_read_flash() & 0x8000)
			{
				break;	
			}
		}	
			
		read_buf_flash(offset, data_read, temp_data_len);
	
		for(uint16_t i = 0; i < num_page; i++)
		{
			page = page + 1;
			
			page_to_buf_flash(page);
			
			while(1)
			{
				if(status_regis_read_flash() & 0x8000)
				{
					break;	
				}
			}	
			
			read_buf_flash(0, &data_read[temp_data_len], BYTE_PER_PAGE);
			
			temp_data_len = temp_data_len + BYTE_PER_PAGE;	
		}
		
		if(temp_data_len < data_len)
		{
			page = page + 1;
			
			page_to_buf_flash(page);
			
			while(1)
			{
				if(status_regis_read_flash() & 0x8000)
				{
					break;	
				}
			}	
				
			read_buf_flash(0, &data_read[temp_data_len], data_len - temp_data_len);
		}
	}
	
return 1;
	
}

	



