/**********************************************************

Software I2C Library for AVR Devices.

Copyright 2008-2012
eXtreme Electronics, India
www.eXtremeElectronics.co.in
**********************************************************/


#ifndef _I2CSOFT_H
#define _I2CSOFT_H

/* 
I/O Configuration 
*/

#define SCLPORT	PORTA	//TAKE PORTA as SCL OUTPUT WRITE
#define SCLDDR	DDRA	//TAKE DDRA as SCL INPUT/OUTPUT configure

#define SDAPORT	PORTA	//TAKE PORTA as SDA OUTPUT WRITE
#define SDADDR	DDRA	//TAKE PORTA as SDA INPUT configure

#define SDAPIN	PINA	//TAKE PORTA TO READ DATA
#define SCLPIN	PINA	//TAKE PORTA TO READ DATA

#define SCL	PORTA4		//PORTA.4 PIN AS SCL PIN
#define SDA	PORTA6		//PORTA.6 PIN AS SDA PIN


#define SOFT_I2C_SDA_LOW	SDADDR|=((1<<SDA))
#define SOFT_I2C_SDA_HIGH	SDADDR&=(~(1<<SDA))

#define SOFT_I2C_SCL_LOW	SCLDDR|=((1<<SCL))
#define SOFT_I2C_SCL_HIGH	SCLDDR&=(~(1<<SCL))

#define DELAY_TCNT	TCNT0

/**********************************************************
SoftI2CInit()

Description:
	Initializes the Soft I2C Engine.
	Must be called before using any other lib functions.
	
Arguments:
	NONE
	
Returns:
	Nothing

**********************************************************/
void SoftI2CInit();	

/**********************************************************
SoftI2CStart()

Description:
	Generates a START(S) condition on the bus.
	NOTE: Can also be used for generating repeat start(Sr)
	condition too.
	
Arguments:
	NONE
	
Returns:
	Nothing

**********************************************************/
void SoftI2CStart();
void SoftI2CStartNonBlocking();
/**********************************************************
SoftI2CStop()

Description:
	Generates a STOP(P) condition on the bus.
	NOTE: Can also be used for generating repeat start
	condition too.
	
Arguments:
	NONE
	
Returns:
	Nothing

**********************************************************/
void SoftI2CStop();
void SoftI2CStopNonBlocking();
/**********************************************************
SoftI2CWriteByte()

Description:
	Sends a Byte to the slave.
	
Arguments:
	8 bit date to send to the slave.
	
Returns:
	non zero if slave acknowledge the data receipt.
	zero other wise.

**********************************************************/
uint8_t SoftI2CWriteByte(uint8_t data);
bool SoftI2CWriteByteNonBlocking(uint8_t data);
/**********************************************************
SoftI2CReadByte()

Description:
	Reads a byte from slave.
	
Arguments:
	1 if you want to acknowledge the receipt to slave.
	0 if you don't want to acknowledge the receipt to slave.
	
Returns:
	The 8 bit data read from the slave.

**********************************************************/
uint8_t SoftI2CReadByte(uint8_t ack);
uint8_t SoftI2CReadByteNonBlocking(bool ack);

#endif 