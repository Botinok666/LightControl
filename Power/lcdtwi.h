/******************************************************************************
This module is used for interfacing with Standard Alpha Numeric LCD Modules.
For More information please see supplied tutorials and videos.
*******************************************************************************/
#ifndef _LCD_H
#define _LCD_H
#include <stddef.h>
/************************************************
	LCD CONNECTIONS
*************************************************/
#define LCD_BL			(1 << 3)
#define LCD_EN			(1 << 2)
#define LCD_RS			(1 << 0)
#define LCD_RW			(1 << 1)

#define LCD_TWI_READ	0x4F
#define LCD_TWI_WR		0x4E

/***************************************************
			F U N C T I O N S
****************************************************/
void LCDInit(void);
void LCDByte(uint8_t, uint8_t);
void LCDBuffer(const uint8_t*, size_t);

/***************************************************
	M A C R O S
***************************************************/
#define LCDCmd(c)	(LCDByte(c,0))
#define LCDData(d)	(LCDByte(d,1))
#endif
