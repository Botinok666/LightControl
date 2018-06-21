/* Bridge.cpp
 * Created: 26.11.2017 22:13:12
 * Version: 1.1
 * Programmed version: 1.1	*/

#include "Bridge.h"
#include <avr/io.h>
#include <avr/interrupt.h>

uint8_t uBuf[64];
volatile bool u1rx = false;
volatile uint8_t destAddr;
volatile uint8_t *u1rxBuf = uBuf, *u0txBuf = uBuf;
uint8_t tBuf[11] = {93,20,0,25,0,111,0,88,0,0x3c,0xab};

ISR (USART0_RX_vect) //Data from RS485
{
	TCNT1 = 0;
	if (UCSR0A & (1 << UPE0))
	{
		UDR1 = 0xFF;
		uBuf[63] = UDR0; //Discard damaged byte
	}
	else
		UDR1 = UDR0; //Retransmit received byte directly because RS485 is slower than RasPi
}

ISR (USART0_TX_vect) //Data into RS485
{
	if (UCSR0B & (1 << TXB80)) //Address has been transmitted
	{
		UCSR0B &= ~(1 << TXB80); //Clear 9th bit
		if ((destAddr & 0x0F) == 3) //Lower nibble equals 3: set config
			UDR0 = *u0txBuf++; //Send first byte from buffer
		else
			U0RXen(); //Enable receiver (we will receive some data)
	}
	else if (u0txBuf <= u1rxBuf) //Transmit all data
	{
		UDR0 = *u0txBuf++;
		TCNT1 = 0;
	}
	else //All data has been transmitted
	{
		UCSR0A = (1 << MPCM0);
		U0RXen(); //Enable receiver (set bus into idle mode)
	}
}

ISR (USART1_RX_vect) //Data from RasPi
{
	if (!u1rx)
	{
		destAddr = UDR1;
		PORTA |= (1 << PORTA3);
		_delay_us(2);
		UCSR0B |= (1 << TXB80);
		UDR0 = destAddr; //Send address over RS485
		if ((destAddr & 0x0F) == 3) //Lower nibble equals 3: set config
		{
			u1rx = true;
			u0txBuf = u1rxBuf = uBuf; //We will receive other data from RasPi and send it over RS485
			TCNT0 = 0;
			TIFR0 |= (1 << TOV0);
			TIMSK0 |= (1 << TOIE0);
		}
		else if (destAddr == 0x12) //For testing purposes
		{
			UCSR1B |= (1 << TXCIE1);
			u0txBuf = tBuf;
			UDR1 = *u0txBuf++;
		}
	}
	else
	{
		TCNT0 = 0;
		*u1rxBuf++ = UDR1;
	}
}

ISR (USART1_TX_vect)
{
	UDR1 = *u0txBuf++;
	if (*u0txBuf == tBuf[sizeof(tBuf) - 1])
		UCSR1B &= ~(1 << TXCIE1);
}

ISR (TIMER0_OVF_vect) //555µs timeout
{
	TIMSK0 &= ~(1 << TOIE0);
	u1rx = false; //Receiving from RasPi was finished
}

inline void mcuInit()
{
	cli();
	//Port A outputs: U0TX, U0EN, U1TX
	DDRA = (1 << DDA1) | (1 << DDA3) | (1 << DDA5);
	//USART 0: 76.8kbps, frame bits: start / 9 data / 2 stop, multi-processor communication
	UBRR0 = 2; //Actual maximum transfer rate: 6400Bps
	UCSR0B = (1 << RXCIE0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << TXEN0) | (1 << UCSZ02); //Interrupts enabled
	UCSR0C = (1 << UPM01) | (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0A = (1 << MPCM0);
	//USART 1: 76.8kbps, frame bits: start / 8 data / no parity / 1 stop
	UBRR1 = 2; //Actual maximum transfer rate: 7680Bps
	UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1); //RX interrupt enabled
	//Timer 0: 460.8kHz clock
	TCCR0B = (1 << CS01);
	//Power reduction
	PRR = (1 << PRTWI) | (1 << PRSPI) | (1 << PRTIM1) | (1 << PRTIM2) | (1 << PRADC);
	sei();
}

int main(void)
{
	mcuInit();
    /* Replace with your application code */
    while (true)
    {
    }
}