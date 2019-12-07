//This module is used for interfacing with HD44780 Alpha Numeric LCD Modules.
#include <avr/io.h>
#include "lcdtwi.h"
#include <util/delay.h>
#include "twi.h"

void LCDnibble(uint8_t nibble) {
	twi_send(nibble | LCD_EN); //Data pins output, RW=0, EN=1
	twi_send(nibble); //Data pins output, RW=0, EN=0
}

void lcd_send_byte(uint8_t c, uint8_t isdata) //Sends one byte to the LCD in 4bit mode
{
	uint8_t hn,ln;	//Nibbles

	hn = c & 0xF0;
	ln = c << 4;

	twi_start();
	twi_send(LCD_TWI_WR);
	if (isdata) //RS=1
	{
		hn |= LCD_RS;
		ln |= LCD_RS;
		twi_send(LCD_RS | LCD_BL); //Data pins 0, RW=0, RS=1
	}
	else
		twi_send(LCD_BL); //Data pins 0, RW=0, RS=0

	LCDnibble(hn | LCD_BL); //Send higher nibble
	LCDnibble(ln | LCD_BL); //Send lower nibble
	twi_stop();
}

void lcd_send_buffer(const uint8_t *buf, uint8_t size) //Sends a buffer of a given size in 4 bit mode
{
	twi_start();
	twi_send(LCD_TWI_WR);
	twi_send(LCD_RS | LCD_BL); //Data pins 0, RW=0, RS=1
	while (size-- > 0)
	{
		LCDnibble((*buf & 0xF0) | LCD_RS | LCD_BL); //Send higher nibble
		LCDnibble((*buf++ << 4) | LCD_RS | LCD_BL); //Send lower nibble
	}
	twi_stop();
}

void lcd_init(void) {
	//This function Initializes the LCD module, must be called before calling LCD related functions
	//Set IO Ports
	twi_start();
	twi_send(LCD_TWI_WR);
	twi_send(0x00);
	twi_stop();
	_delay_ms(666); //After power on wait for LCD to initialize
	twi_start();
	twi_send(LCD_TWI_WR);
	LCDnibble(0x20 | LCD_BL);
	twi_stop();
	_delay_ms(1);
	lcd_send_byte(0x28, 0);
	lcd_send_byte(0x0C, 0);
}
