#include <stdio.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"



#define WRITE_BUFFER1					0x84
#define WRITE_BUF_TO_PAGE1		0x83
#define DUMMY									0x00

#define Main_Memory_Page_Read	0x53

#define BUF1_READ_LOW					0xD1	// Low Frequency
#define BUF1_READ_HIGH				0xD4	// High Frequency
#define BUF1_READ_LEGACY			0x54	// Legacy Commands
#define STATUS_REGIS_READ			0xD7	/** Status Register Read [bit 7]
																			* 1:Device is ready 
																			* 0:Device is busy with an internal operation **/

#define BYTE_PER_PAGE					264
#define MAX_PAGE							4096
#define BYTES_MAX							BYTE_PER_PAGE * MAX_PAGE


/** Ready Status **/
int16_t status_regis_read_flash(void);

/** Write Flash **/
int8_t write_buf_flash(uint16_t addr, uint8_t* data_wirte, uint32_t data_len);
int8_t write_buf_to_page_flash(uint16_t page);

/** Read Flash **/
int8_t page_to_buf_flash(uint16_t page);
int8_t read_buf_flash(uint16_t addr, uint8_t* data_read, uint32_t data_len);

/** Write & Read Flash **/
int8_t write_flash(uint32_t addr, uint8_t* data_wirte, uint32_t data_lan);
int8_t read_flash(uint32_t addr, uint8_t* data_read, uint32_t data_lan);
