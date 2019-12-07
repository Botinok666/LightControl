#include "stdint-gcc.h"
#ifndef _LCD_H
#define _LCD_H
/************************************************
	LCD CONNECTIONS
*************************************************/
#define F_CPU 8000000UL

#define LCD_BL			(1 << 3)
#define LCD_EN			(1 << 2)
#define LCD_RS			(1 << 0)
#define LCD_RW			(1 << 1)

#define LCD_TWI_READ	0x4F
#define LCD_TWI_WR		0x4E

/***************************************************
			F U N C T I O N S
****************************************************/
void lcd_init(void);
void lcd_send_byte(uint8_t, uint8_t);
void lcd_send_buffer(const uint8_t*, uint8_t);

#endif
