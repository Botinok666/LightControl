/**********************************************************

Software I2C Library for AVR Devices.

Copyright 2008-2012
eXtreme Electronics, India
www.eXtremeElectronics.co.in
**********************************************************/
 
#include <avr/io.h>
#include "AC.h"

#include "i2csoft.h"

#define Q_DEL _delay_loop_2(3)
#define H_DEL _delay_loop_2(5)

#define Q_DEL_NB 13
#define H_DEL_NB (Q_DEL_NB * 2)

void SoftI2CInit()
{
	SDAPORT &= (1 << SDA);
	SCLPORT &= (1 << SCL);
	
	SOFT_I2C_SDA_HIGH;	
	SOFT_I2C_SCL_HIGH;	
}
void SoftI2CStart()
{
	SOFT_I2C_SCL_HIGH;
	H_DEL;
	
	SOFT_I2C_SDA_LOW;	
	H_DEL;  	
}
void SoftI2CStartNonBlocking()
{
	/*DELAY_TCNT = 0;
	SOFT_I2C_SCL_HIGH;
	while (DELAY_TCNT < H_DEL_NB);*/
	//Assume that SCL high
	DELAY_TCNT = 0;
	SOFT_I2C_SDA_LOW;
	while (DELAY_TCNT < H_DEL_NB);
	
	DELAY_TCNT = 0;
	SOFT_I2C_SCL_LOW;
	while (DELAY_TCNT < Q_DEL_NB);
}

void SoftI2CStop()
{
	 SOFT_I2C_SDA_LOW;
	 H_DEL;
	 SOFT_I2C_SCL_HIGH;
	 Q_DEL;
	 SOFT_I2C_SDA_HIGH;
	 H_DEL;
}

void SoftI2CStopNonBlocking()
{
	//SCL low
	DELAY_TCNT = 0;
	SOFT_I2C_SDA_LOW;
	while (DELAY_TCNT < H_DEL_NB);
	
	DELAY_TCNT = 0;
	SOFT_I2C_SCL_HIGH;
	while (DELAY_TCNT < Q_DEL_NB);
	
	DELAY_TCNT = 0;
	SOFT_I2C_SDA_HIGH;
	while (DELAY_TCNT < H_DEL_NB);
}

bool SoftI2CWriteByteNonBlocking(uint8_t data)
{
	//SCL low, SDA low
	for (uint8_t i = 0; i < 8; i++)
	{
		DELAY_TCNT = 0;
		if (data & 0x80)
			SOFT_I2C_SDA_HIGH;
		else
			SOFT_I2C_SDA_LOW;
		while (DELAY_TCNT < Q_DEL_NB);
		
		DELAY_TCNT = 0;
		SOFT_I2C_SCL_HIGH;
		while (DELAY_TCNT < Q_DEL_NB);
		while ((SCLPIN & (1 << SCL)) == 0 && DELAY_TCNT >= Q_DEL_NB); //Clock stretching
		DELAY_TCNT = 0;
		while (DELAY_TCNT < Q_DEL_NB);

		DELAY_TCNT = 0;
		SOFT_I2C_SCL_LOW;
		while (DELAY_TCNT < Q_DEL_NB);
				
		data <<= 1;
	}
	//SCL low
	//The 9th clock (ACK Phase)
	DELAY_TCNT = 0;
	SOFT_I2C_SDA_HIGH;
	while (DELAY_TCNT < Q_DEL_NB);
	
	DELAY_TCNT = 0;
	SOFT_I2C_SCL_HIGH;
	while (DELAY_TCNT < Q_DEL_NB);
	while ((SCLPIN & (1 << SCL)) == 0 && DELAY_TCNT >= Q_DEL_NB); //Clock stretching
	DELAY_TCNT = 0;
	while (DELAY_TCNT < Q_DEL_NB);
	
	bool ack = !(SDAPIN & (1 << SDA));
	
	DELAY_TCNT = 0;
	SOFT_I2C_SCL_LOW;
	while (DELAY_TCNT < H_DEL_NB);
	
	return ack;
	//SCL low, SDA high
}

uint8_t SoftI2CWriteByte(uint8_t data)
{
	 
	 uint8_t i;
	 	
	 for(i=0;i<8;i++)
	 {
		SOFT_I2C_SCL_LOW;
		Q_DEL;
		
		if(data & 0x80)
			SOFT_I2C_SDA_HIGH;
		else
			SOFT_I2C_SDA_LOW;	
		
		H_DEL;
		
		SOFT_I2C_SCL_HIGH;
		H_DEL;
		
		while((SCLPIN & (1<<SCL))==0);
			
		data=data<<1;
	}
	 
	//The 9th clock (ACK Phase)
	SOFT_I2C_SCL_LOW;
	Q_DEL;
		
	SOFT_I2C_SDA_HIGH;		
	H_DEL;
		
	SOFT_I2C_SCL_HIGH;
	H_DEL;	
	
	uint8_t ack=!(SDAPIN & (1<<SDA));
	
	SOFT_I2C_SCL_LOW;
	H_DEL;
	
	return ack;
	 
}

uint8_t SoftI2CReadByteNonBlocking(bool ack)
{
	//SCL low, SDA high
	uint8_t data = 0;
	for(uint8_t i = 0; i < 8; i++)
	{
		DELAY_TCNT = 0;
		SOFT_I2C_SCL_HIGH;
		while (DELAY_TCNT < Q_DEL_NB);
		while ((SCLPIN & (1 << SCL)) == 0 && DELAY_TCNT >= Q_DEL_NB); //Clock stretching
		DELAY_TCNT = 0;
		while (DELAY_TCNT < Q_DEL_NB);
		
		data <<= 1;
		if (SDAPIN & (1 << SDA))
			data |= 1;		
			
		DELAY_TCNT = 0;
		SOFT_I2C_SCL_LOW;
		while (DELAY_TCNT < H_DEL_NB);
	}
	
	DELAY_TCNT = 0;
	if (ack)
		SOFT_I2C_SDA_LOW;
	else
		SOFT_I2C_SDA_HIGH;
	while (DELAY_TCNT < Q_DEL_NB);
	
	DELAY_TCNT = 0;
	SOFT_I2C_SCL_HIGH;
	while (DELAY_TCNT < Q_DEL_NB);
	while ((SCLPIN & (1 << SCL)) == 0 && DELAY_TCNT >= Q_DEL_NB); //Clock stretching
	DELAY_TCNT = 0;
	while (DELAY_TCNT < Q_DEL_NB);
	
	DELAY_TCNT = 0;
	SOFT_I2C_SCL_LOW;
	while (DELAY_TCNT < Q_DEL_NB);
	
	DELAY_TCNT = 0;
	SOFT_I2C_SDA_HIGH;
	while (DELAY_TCNT < Q_DEL_NB);
	
	return data;
	//SCL low, SDA high
} 

uint8_t SoftI2CReadByte(uint8_t ack)
{
	uint8_t data=0x00;
	uint8_t i;
			
	for(i=0;i<8;i++)
	{
			
		SOFT_I2C_SCL_LOW;
		H_DEL;
		SOFT_I2C_SCL_HIGH;
		H_DEL;
			
		while((SCLPIN & (1<<SCL))==0);
		
		if(SDAPIN &(1<<SDA))
			data|=(0x80>>i);
			
	}
		
	SOFT_I2C_SCL_LOW;
	Q_DEL;						//Soft_I2C_Put_Ack
	
	if(ack)
	{
		SOFT_I2C_SDA_LOW;	
	}
	else
	{
		SOFT_I2C_SDA_HIGH;
	}	
	H_DEL;
	
	SOFT_I2C_SCL_HIGH;
	H_DEL;
	
	SOFT_I2C_SCL_LOW;
	H_DEL;
			
	return data;
	
}
