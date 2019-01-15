//This module is used for interfacing with HD44780 Alpha Numeric LCD Modules.
#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include "twi.h"
#include "lcdtwi.h"

void LCDnibble(uint8_t nibble) {
	twi_send(nibble | LCD_EN); //Data pins output, RW=0, EN=1
	twi_send(nibble); //Data pins output, RW=0, EN=0
}

void LCDByte(uint8_t c, uint8_t isdata) //Sends one byte to the LCD in 4bit mode
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
void LCDBuffer(const uint8_t *buf, size_t size) //Sends a buffer of a given size in 4 bit mode
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

void LCDInit(void) {
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
	LCDCmd(0x28);
	LCDCmd(0x0C);
}
