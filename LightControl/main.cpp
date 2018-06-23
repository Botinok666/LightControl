/* Air conditioning Mk1
 * Created: 24.11.2017 19:17:15
 * Version: 1.2
 * Programmed version: 1.1	*/

#include "AC.h"
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/crc16.h>

volatile uint8_t tachCnt, txCnt, rxMode = 0, cycles;
volatile uint16_t tachPrev, adcSum, eeSave = 0;
uint32_t tachSum;
uint8_t *txBuf;

struct sysConfig
{
	uint8_t fanLevelOverride;
	int16_t minDeltaRH, dDeltaRH;
	int16_t minDeltaT, dDeltaT;

	uint16_t CRC16;
} validConf, rcvdConf;

struct sysStatus
{
	uint8_t fanLevel;
	uint16_t rpmFront, rpmRear;
	uint16_t currentDraw;
	int16_t insideRH, outsideRH;
	int16_t insideT, outsideT;

	uint16_t CRC16;
} validStatus, tmpStatus;

union am2302 {
	struct Frame {
		int16_t RH;
		int16_t T;
		uint8_t checksum;
	} frame;
	uint8_t arr[sizeof(Frame)];
} AM2302data;

sysConfig EEMEM eConf = { 0xFF, 100, 400, 40, 80, 0 };

void inline U0TXen()
{
	PORTA |= (1 << PORTA5);
	_delay_us(39);
}

uint16_t CalculateCRC16(void *arr, int8_t count)
{
	uint8_t *ptr = (uint8_t*)arr;
	uint16_t CRC16 = 0xffff;
	while (--count >= 0)
		CRC16 = _crc_xmodem_update(CRC16, *ptr++);
	return CRC16;
}

void FanRegulation()
{
	//Upper bounds for regulation
	int16_t dT = tmpStatus.insideT - tmpStatus.outsideT - validConf.minDeltaT;
	int16_t dRH = tmpStatus.insideRH - tmpStatus.outsideRH - validConf.minDeltaRH;
	int16_t A = (FanMax - FanMin) * dT / validConf.dDeltaT;
	int16_t B = (FanMax - FanMin) * dRH / validConf.dDeltaRH;
	if (B > A)
		A = B;
	A += FanMin;
	if (A > tmpStatus.fanLevel) //At least one of upper bounds is above
	{
		//tmpStatus.fanLevel = A > FanMax ? FanMax : A; //Increase level
	}
	else
	{
		//Lower bounds for regulation
		dT -= validConf.minDeltaT >> 3;
		dRH -= validConf.minDeltaRH >> 3;
		A = (FanMax - FanMin) * dT / validConf.dDeltaT;
		B = (FanMax - FanMin) * dRH / validConf.dDeltaRH;
		if (B > A)
			A = B;
		A += FanMin;
		//if (A < tmpStatus.fanLevel) //Both lower bounds are below
		//	tmpStatus.fanLevel = A > FanMin ? A : 0; //Decrease level
	}
}

ISR (USART0_RX_vect)
{
	static char uCnt = 0;
	static uint8_t *rxBuf = (uint8_t*)&rcvdConf;
	uint8_t data = UDR0;
	if (UCSR0A & (1 << MPCM0)) //Address listening mode
	{
		if (CmdLC <= data && data <= CmdUC)
		{
			UCSR0A = 0; //Clear MPCM bit
			rxMode = data;
			if (data == SetConfig)
			{
				uCnt = sizeof(sysConfig); //Bytes to receive
				rxBuf = (uint8_t*)&rcvdConf; //First byte address in structure
				TCNT0 = 0;
				TCCR0B = (1 << CS02); //14.4kHz clock
				TIFR0 = (1 << OCF0B); //Clear interrupt flag
				TIMSK0 = (1 << OCIE0B); //Enable packet lost interrupt
			}
			else
			{
				U0TXen();
				if (rxMode == GetConfig)
				{
					txCnt = sizeof(sysConfig) - 1; //Because one byte will be send right there
					txBuf = (uint8_t*)&validConf;
				}
				else //Get status
				{
					tmpStatus.CRC16 = CalculateCRC16(&tmpStatus, sizeof(sysStatus) - 2);
					memcpy(&validStatus, &tmpStatus, sizeof(sysStatus));
					txCnt = sizeof(sysStatus) - 1;
					txBuf = (uint8_t*)&validStatus;
				}
				UDR0 = *txBuf++; //Send first byte
			}
		}
		else
			rxMode = 0;
	}
	else if (rxMode == SetConfig)
	{
		*rxBuf++ = data;
		if (--uCnt == 0) //Packet received
		{
			TIMSK0 = 0; //Disable packet lost interrupt
			TCCR0B = (1 << CS01); //460.8kHz clock
			if (CalculateCRC16(&rcvdConf, sizeof(sysConfig) - 2) == rcvdConf.CRC16) //CRC OK
				memcpy(&validConf, &rcvdConf, sizeof(sysConfig));
		}
	}
}

ISR (USART0_TX_vect) //Transmit to RS485
{
	if (txCnt--)
		UDR0 = *txBuf++; //Send next character from the given buffer
	else
	{
		U0RXen();
		UCSR0A = (1 << MPCM0); //Set MPCM bit
		rxMode = 0;
	}
}

ISR (TIMER0_COMPB_vect) //Packet lost interrupt
{
	TIMSK0 = 0; //Disable packet lost interrupt
	TCCR0B = (1 << CS01); //460.8kHz clock
	UCSR0A = (1 << MPCM0);
	rxMode = 0;
}

ISR (TIMER1_OVF_vect) //Occurs every 142.2ms
{
	if (cycles++ >= 15)
	{
		cycles = 0;
		eeSave += 8;
		if (eeSave == 0)
			eeprom_update_block(&validConf, &eConf, sizeof(sysConfig));
	}
	switch (cycles)
	{
		case 0:
		case 8:
		//SelChA();
		tachCnt = 0;
		tachSum = 0;
		TIMSK1 |= (1 << ICIE1);
	break;

		case 3:
		case 15:
		PORTB &= ~(1 << PORTB2);
		DDRB |= (1 << DDB2); //Interface pulled low
		_delay_ms(1.666);
		DDRB &= ~(1 << DDB2); //Interface released
		PCMSK1 = (1 << PCINT10);
		GIFR = (1 << PCIF1);
		GIMSK = (1 << PCIE1);
	break;

		case 4:
		case 12:
		//SelChB();
		tachCnt = 0;
		tachSum = 0;
		TIMSK1 |= (1 << ICIE1);
	break;

		case 2:
		case 6:
		case 10:
		case 14:
		TIMSK1 &= ~(1 << ICIE1);
		if (cycles == 2 || cycles == 10)
			tmpStatus.rpmFront = 0;
		else
		{
			tmpStatus.rpmRear = 0;
			tmpStatus.currentDraw = (adcSum << 3) / Idiv_x1mA;
			adcSum = 0;
		}
		if (tachCnt > 1)
		{
			uint32_t temp = (F_CPU / 8) * 30 * (tachCnt - 1);
			temp /= tachSum;
			if (cycles < 10)
				tmpStatus.rpmFront = temp;
			else
				tmpStatus.rpmRear = temp;
		}
	break;
	}
	ADCSRA |= (1 << ADSC);
	uint8_t level = validConf.fanLevelOverride < FanMax ?
		(((uint16_t)validConf.fanLevelOverride * PWM_max) >> 8) : validStatus.fanLevel;
	if (OCR2AL > level)
	{
		OCR2AL--;
		if (OCR2AL < FanMin)
			OCR2AL = 0;
	}
	else if (OCR2AL < level)
	{
		OCR2AL++;
		if (OCR2AL < FanMin)
			OCR2AL = level;
	}
	OCR2BL = ICR2L - OCR2AL;
}

ISR (PCINT1_vect)
{
	GIMSK = 0;
	_delay_us(80);
	//Check start condition 2
	tmpStatus.fanLevel = 1;
	if(!(PINB & (1 << PINB2)))
		return;
	_delay_us(80);
	//read the data
	for (uint8_t j = 0; j < 5; j++) //read 5 byte
	{
		uint8_t result = 0;
		for (uint8_t i = 0; i < 8; i++) //read every bit
		{
			uint8_t timeoutcounter = 0;
			while (!(PINB & (1 << PINB2))) //wait for an high input
			{
				_delay_us(10);
				if (++timeoutcounter > 5) //timeout
					return;
			}
			_delay_us(30);
			if (PINB & (1 << PINB2)) //if input is high after 30 us, get result
				result |= (1 << (7 - i));
			timeoutcounter = 0;
			while (PINB & (1 << PINB2)) //wait until input get low (non blocking)
			{
				_delay_us(10);
				if (++timeoutcounter > 5) //timeout
					return;
			}
		}
		AM2302data.arr[j] = result;
	}
	tmpStatus.fanLevel = 2;
	if ((uint8_t)(AM2302data.arr[0] + AM2302data.arr[1] + AM2302data.arr[2] + AM2302data.arr[3]) != AM2302data.arr[4])
		return;
	if (AM2302data.frame.T < 0)
		AM2302data.frame.T = ~(AM2302data.frame.T & 0x7FFF) + 1;
	if (cycles == 3)
	{
		tmpStatus.insideRH = AM2302data.frame.RH;
		tmpStatus.insideT = AM2302data.frame.T;
	}
	else
	{
		tmpStatus.outsideRH = AM2302data.frame.RH;
		tmpStatus.outsideT = AM2302data.frame.T;
		FanRegulation();
	}
	tmpStatus.fanLevel = 3;
}

ISR (TIMER1_CAPT_vect)
{
	if (tachCnt++ > 0)
	{
		if (ICR1 < tachPrev)
			tachSum += 0xFFFF - tachPrev + ICR1;
		else
			tachSum += ICR1 - tachPrev;
	}
	if (tachCnt > 7)
		TIMSK1 &= ~(1 << ICIE1);
	tachPrev = ICR1;
}

ISR (ADC_vect)
{
	adcSum += ADC;
}


void inline mcuInit()
{
	cli();
	//Port A outputs: U0TX, PWMA, PWMB, U0EN, SEL
	DDRA = (1 << DDA1) | (1 << DDA3) | (1 << DDA4) | (1 << DDA5) | (1 << DDA6);
	//USART 0: 76.8kbps, frame bits: start / 9 data / 2 stop, multi-processor communication
	UBRR0 = 2; //Actual maximum transfer rate: 6400Bps
	UCSR0B = (1 << RXCIE0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << TXEN0) | (1 << UCSZ02);
	UCSR0C = (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0A = (1 << MPCM0);
	//Timer 0: 460.8kHz clock, CTC on OCR0A
	//TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01); //1 tick = 2.17µs
	//OCR0A = 0xFE;
	OCR0B = (sizeof(sysStatus) > sizeof(sysConfig) ? sizeof(sysStatus) : sizeof(sysConfig)) * 234 / 69; //Equals 57
	//Timer 1: 460.8kHz clock, input capture on leading edge, noise filtering, OVF interrupt
	//If fan is running for 4 poles, minimum measurable speed is 211rpm
	TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11);
	TIMSK1 = (1 << TOIE1);
	//Timer 2: 3.6864MHz clock, fast PWM, top in ICR2, OC2A non-inverting (TOCC3), OC2B inverting (TOCC2)
	ICR2 = PWM_max; //25kHz PWM frequency
	OCR2A = 0; //Non-inverting output: off
	OCR2B = PWM_max; //Inverting output: off
	TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM21);
	TCCR2B = (1 << WGM23) | (1 << WGM22) | (1 << CS20);
	TOCPMSA0 |= (1 << TOCC2S1) | (1 << TOCC3S1);
	TOCPMCOE |= (1 << TOCC2OE) | (1 << TOCC3OE);
	//ADC: 1.1V reference, 115.2kHz clock, ADC0 input, interrupt
	ADMUXB = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS0);
	DIDR0 = (1 << ADC0D);
	//Power reduction: I²C, USART1 and SPI are not used in this project
	PRR = (1 << PRTWI) | (1 << PRUSART1) | (1 << PRSPI);
	sei();
}

int main(void)
{
	mcuInit();

	eeprom_read_block(&validConf, &eConf, sizeof(sysConfig));
	validConf.fanLevelOverride = 0xFF; //No override at startup
	validConf.CRC16 = CalculateCRC16(&validConf, sizeof(sysConfig) - 2);
    /* Replace with your application code */
    while (1) { }
}