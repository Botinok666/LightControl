/* Bridge.cpp
 * Created: 26.11.2017 22:13:12
 * Version: 2.0
 * Programmed version: 1.2	*/

#include "Bridge.h"
#include <avr/io.h>
#include <avr/interrupt.h>

uint8_t uBuf[64];
volatile bool u1rx = false;
volatile uint8_t destAddr;
volatile uint8_t rxIdx, txIdx;

ISR (USART0_RX_vect) //Data from RS485
{
	UDR1 = UDR0; //Retransmit received byte directly because RS485 is slower than RasPi
}

ISR (USART0_TX_vect) //Data into RS485
{
	if (UCSR0B & (1 << TXB80)) //Address has been transmitted
	{
		UCSR0B &= ~(1 << TXB80); //Clear 9th bit
		if ((destAddr & 0x0F) != 3) //Lower nibble equals 3: set config
			U0RXen(); //Enable receiver (we will receive some data)
	}
	else if (txIdx < rxIdx) //Transmit data from RasPi
		UDR0 = uBuf[txIdx++];
}

ISR (USART1_RX_vect) //Data from RasPi
{
	if (!u1rx)
	{
		destAddr = UDR1;
		U0TXen();
		_delay_us(2);
		UCSR0B |= (1 << TXB80);
		UDR0 = destAddr; //Send address over RS485
		if ((destAddr & 0x0F) == 3) //Lower nibble equals 3: set config
		{
			u1rx = true;
			txIdx = rxIdx = 0; //We will receive other data from RasPi and send it over RS485
			TCNT0 = 0;
			TIFR0 = (1 << OCF0B);
			TIMSK0 = (1 << OCIE0B);
		}
	}
	else
	{
		TCNT0 = 0;
		uBuf[rxIdx++] = UDR1;
		if ((UCSR0A & (1 << UDRE0)) && txIdx < rxIdx)
			UDR0 = uBuf[txIdx++];
	}
}

ISR (TIMER0_COMPB_vect) //2.66ms timeout
{
	TIMSK0 = 0;
	u1rx = false; //Receiving from RasPi was finished
	U0RXen(); //Enable receiver (set bus into idle mode)
}

inline void mcuInit()
{
	cli();
	//Port A outputs: U0TX, U0EN, U1TX
	DDRA = (1 << DDA1) | (1 << DDA3) | (1 << DDA5);
	//USART 0: 76.8kbps, frame bits: start / 9 data / 2 stop, multi-processor communication
	UBRR0 = 2; //Actual maximum transfer rate: 6400Bps
	UCSR0B = (1 << RXCIE0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << TXEN0) | (1 << UCSZ02); //Interrupts enabled
	UCSR0C = (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00);
	//USART 1: 76.8kbps, frame bits: start / 8 data / no parity / 1 stop
	UBRR1 = 2; //Actual maximum transfer rate: 7680Bps
	UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1); //RX interrupt enabled
	//Timer 0: 57.6kHz clock
	TCCR0B = (1 << CS01) | (1 << CS00); //~17.36µs tick
	OCR0B = 154; //~2.66ms timeout
	//Timer 1: 460.8kHz, phase and frequency correct, top in ICR1
	TCCR1B = (1 << WGM13) | (1 << CS11);
	ICR1 = 576; //400Hz output
	TOCPMSA1 = (1 << TOCC6S0); //OC1B output to TOCC6
	TOCPMCOE = (1 << TOCC6OE); //TOCC6 output to PA7
	//ADC: 115.2kHz, channel 0, 2.2V reference
	ADMUXB = (1 << REFS1);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0);
	DIDR0 = (1 << ADC0D);
	//Power reduction
	PRR = (1 << PRTWI) | (1 << PRSPI) | (1 << PRTIM2);
	sei();
}

int main(void)
{
	mcuInit();
    /* Replace with your application code */
    while (true) { }
}
