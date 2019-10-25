/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "spi_flash.h"
#include "spi_sw.h"

void test_flash_1(void)
{
	uint16_t page 					=	0x02; 	 //0x1A;
	uint16_t addr 					= 0; //0x85;

	uint8_t data_write[3] 	= {0x7C, 0x8D, 0x9F};
	uint8_t data_read[3];
		
	write_buf_flash(addr, data_write, sizeof(data_write));
	write_buf_to_page_flash(page);
		
	while(1)
	{	
		if(status_regis_read_flash() & 0x8000)
		{
			NRF_LOG_INFO("rdy");
			NRF_LOG_FLUSH();
			break;	
		}
	}
			
	page_to_buf_flash(page);
	while(1)
		{
			if(status_regis_read_flash() & 0x8000)
			{
				break;	
			}
		}
	read_buf_flash(addr, data_read, sizeof(data_read));
	
	for (uint32_t k = 0; k < sizeof(data_write); k++)
		{
			
			if(data_write[k] == data_read[k])
			{
			}
			else
			{
				NRF_LOG_INFO("data_%d is 0", k);
				NRF_LOG_FLUSH();
			}
		}
	
		NRF_LOG_INFO("Finish Write-Read Testing");
		NRF_LOG_FLUSH();
	
}

void test_flash_2(void)
{	
		uint8_t data_write[1];
		uint8_t data_read[1];
		uint32_t addr = 0;
		
	
		for(uint32_t i = 0; i < sizeof(data_write); i++)
		{
			data_write[i] = i;
		}

		memset(data_read, 0, sizeof(data_read));
		
		write_flash(addr, data_write, sizeof(data_write));
		read_flash(addr, data_read, sizeof(data_read));
		
	
		for (uint32_t k = 0; k < sizeof(data_write); k++)
		{
			
			if(data_write[k] == data_read[k])
			{
			}
			else
			{
				NRF_LOG_INFO("data_%d is 0", k);
				NRF_LOG_FLUSH();
			}
		}
	
		NRF_LOG_INFO("Finish Write-Read Testing");
		NRF_LOG_FLUSH();	
}


void print_byte_of_page(void)
{
	 NRF_LOG_INFO("%d", (status_regis_read_flash() & 0x100)? 1:0);
	 NRF_LOG_FLUSH();
}

int main(void)
{
    bsp_board_leds_init();
		init_spi_sw();
		
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    NRF_LOG_INFO("SPI example.");
		NRF_LOG_FLUSH();
		
//		print_byte_of_page();
		test_flash_2();
		
    while (1)
    {
        
				//nrf_delay_ms(100);
    }
}
