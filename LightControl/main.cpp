/* Air conditioning Mk1
 * Created: 24.11.2017 19:17:15
 * Version: 1.1
 * Programmed version: 1.1	*/

#include "AC.h"
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/crc16.h>

volatile bool tReq, tRdy, sRdy, amRead, dRdy = false;
volatile uint8_t tachCnt, txCnt, rxMode = 0, cycles;
volatile uint16_t tachPrev, adcSum;
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
				OCR0A = OCR0B; //Load cycles count
				TIFR0 |= (1 << OCF0A); //Clear interrupt flag
				TIMSK0 |= (1 << OCIE0A); //Enable packet lost interrupt
			}
			else
				tReq = true; //Data transmit request
		}
		else
			rxMode = 0;
	}
	else if (rxMode == SetConfig)
	{
		*rxBuf++ = data;
		if (--uCnt == 0) //Packet received
		{
			dRdy = true;
			TIMSK0 = 0; //Disable packet lost interrupt
			TCCR0B = (1 << CS01); //460.8kHz clock
			OCR0A = 0xFE;
		}
	}
}

ISR (USART0_UDRE_vect) //Transmit to RS485
{
	if (--txCnt)
		UDR0 = *txBuf++; //Send next character from the given buffer
	else
	{
		U0RXen();
		UCSR0A = (1 << MPCM0); //Set MPCM bit
		rxMode = 0;
	}
}

ISR (TIMER0_COMPA_vect) //Packet lost interrupt
{
	TIMSK0 = 0; //Disable packet lost interrupt
	TCCR0B = (1 << CS01); //460.8kHz clock
	OCR0A = 0xFE;
	UCSR0A = (1 << MPCM0);
	rxMode = 0;
}

ISR (TIMER1_OVF_vect) //Occurs every 142.2ms
{
	if (++cycles > 7)
		cycles = 0;
	switch (cycles)
	{
		case 0:
		SelChA();
		tachCnt = 0;
		tachSum = 0;
		TIMSK1 |= (1 << ICIE1);
		break;

		case 2:
		TIMSK1 &= ~(1 << ICIE1);
		if (tachCnt < 2)
			tmpStatus.rpmFront = 0;
		else
			tRdy = true;
		break;

		case 3:
		case 7:
		amRead = true;
		break;

		case 4:
		SelChB();
		tachCnt = 0;
		tachSum = 0;
		TIMSK1 |= (1 << ICIE1);
		break;

		case 6:
		TIMSK1 &= ~(1 << ICIE1);
		if (tachCnt < 2)
			tmpStatus.rpmRear = 0;
		else
			tRdy = true;
		tmpStatus.currentDraw = (adcSum << 3) / Idiv_x1mA;
		adcSum = 0;
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
	{
		tRdy = true;
		TIMSK1 &= ~(1 << ICIE1);
	}
	tachPrev = ICR1;
}

ISR (ADC_vect)
{
	adcSum += ADC;
}

void inline U0TXen()
{
	PORTA |= (1 << PORTA5);
	_delay_us(2);
}

uint16_t CalculateCRC16(void *arr, int8_t count)
{
	uint8_t *ptr = (uint8_t*)arr;
	uint16_t CRC16 = 0;
	while (--count >= 0)
		CRC16 = _crc_xmodem_update(CRC16, *ptr++);
	return CRC16;
}

void inline ResponseProcessing()
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
		validStatus = tmpStatus;
		txCnt = sizeof(sysStatus) - 1;
		txBuf = (uint8_t*)&validStatus;
	}
	UDR0 = *txBuf++; //Send first byte
}

bool AM2302read()
{
	int8_t nBits;
	DOPullLow();
	_delay_ms(2);
	DORelease();
	TCNT0 = 0;
	TIFR0 |= (1 << OCF0A);
	while ((PINB & (1 << PINB2)) && ~(TIFR0 & (1 << OCF0A))); //Low response
	for (nBits = -1; nBits < 40; nBits++)
	{
		TCNT0 = 0;
		while (~(PINB & (1 << PINB2)) && ~(TIFR0 & (1 << OCF0A))); //Wait for high level
		while ((PINB & (1 << PINB2)) && ~(TIFR0 & (1 << OCF0A))); //Wait for low level
		if (TIFR0 & (1 << OCF0A)) //Sensor not responded
			return false;
		int8_t tmp = TCNT0 < 45 ? 0 : 1; //Less than 98µs - 0
		if (nBits < 16)
			AM2302data.frame.RH = (AM2302data.frame.RH << 1) | tmp;
		else if (nBits < 32)
			AM2302data.frame.T = (AM2302data.frame.T << 1) | tmp;
		else
			AM2302data.frame.checksum = (AM2302data.frame.checksum << 1) | tmp;
	}
	uint8_t cSum = 0;
	for (nBits = 0; nBits < 4; nBits++)
		cSum += AM2302data.arr[nBits];
	if (cSum != AM2302data.frame.checksum)
		return false;
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
		sRdy = true;
	}
	amRead = false;
	return true;
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
		tmpStatus.fanLevel = A > FanMax ? FanMax : A; //Increase level
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
		if (A < tmpStatus.fanLevel) //Both lower bounds are below
			tmpStatus.fanLevel = A > FanMin ? A : 0; //Decrease level
	}
}

void inline mcuInit()
{
	cli();
	//Port A outputs: U0TX, PWMA, PWMB, U0EN, SEL
	DDRA = (1 << DDA1) | (1 << DDA3) | (1 << DDA4) | (1 << DDA5) | (1 << DDA6);
	//USART 0: 76.8kbps, frame bits: start / 9 data / 2 stop, multi-processor communication
	UBRR0 = 2; //Actual maximum transfer rate: 6400Bps
	UCSR0B = (1 << RXCIE0) | (1 << UDRE0) | (1 << RXEN0) | (1 << TXEN0) | (1 << UCSZ02);
	UCSR0C = (1 << UPM01) | (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0A = (1 << MPCM0);
	//Timer 0: 460.8kHz clock, CTC on OCR0A
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01); //1 tick = 2.17µs
	OCR0A = 0xFE;
	OCR0B = (sizeof(sysStatus) > sizeof(sysConfig) ? sizeof(sysStatus) : sizeof(sysConfig)) * 234 / 69;
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

	eeprom_read_block(&validConf, &eConf, sizeof(sysConfig));
	validConf.fanLevelOverride = 0xFF; //No override at startup
	validConf.CRC16 = CalculateCRC16(&validConf, sizeof(sysConfig) - 2);
	sei();
}

int main(void)
{
	uint8_t readTries = 0;
	mcuInit();
    /* Replace with your application code */
    while (1)
    {
		if (tReq) //Data must be sent over RS485
		{
			ResponseProcessing();
			tReq = false;
		}

		if (dRdy) //Command packet acquired
		{
			if (CalculateCRC16(&rcvdConf, sizeof(sysConfig) - 2) == rcvdConf.CRC16) //CRC OK
			{
				validConf = rcvdConf;
				eeprom_update_block(&validConf, &eConf, sizeof(sysConfig));
			}
			dRdy = false;
		}

		if (tRdy) //Calculate and store RPM
		{
			uint32_t temp = (F_CPU / 8) * 30 * (tachCnt - 1);
			temp /= tachSum;
			if (cycles < 4)
				tmpStatus.rpmFront = temp;
			else
				tmpStatus.rpmRear = temp;
			tRdy = false;
		}

		if (sRdy)
		{
			FanRegulation();
			sRdy = false;
		}

		if (amRead && !rxMode) //Read data from AM2302
		{
			if (!AM2302read() && readTries++ > 3)
			{
				readTries = 0;
				amRead = false;
				if (cycles == 3) //Set special values to indicate an error on that channel
				{
					tmpStatus.insideRH = 0;
					tmpStatus.insideT = 850;
				}
				else
				{
					tmpStatus.outsideRH = 0;
					tmpStatus.outsideT = 850;
				}
			}
			else
				readTries = 0;
		}
    }
}