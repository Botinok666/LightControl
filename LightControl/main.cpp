/* Air conditioning Mk1
 * Created: 24.11.2017 19:17:15
 * Version: 1.4
 * Programmed version: 1.4	*/

#include "AC.h"
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/crc16.h>

volatile uint8_t tachCnt, tachOvf, txCnt, rxMode = 0, rxMark, amCnt, fanLvl;
volatile uint16_t tachPrev, adcSum, cycles = 0;
uint32_t tachSum;
uint8_t *txBuf;

struct sysConfig
{
	uint8_t fanLevelOverride;
	int16_t minDeltaRH, dDeltaRH;
	int16_t minDeltaT, dDeltaT;

	uint16_t CRC16;
} validConf;

struct sysStatus
{
	uint8_t fanLevel;
	uint16_t rpmFront, rpmRear;
	uint16_t currentDraw;
	int16_t insideRH, outsideRH;
	int16_t insideT, outsideT;

	uint16_t CRC16;
} tmpStatus;

union am2302 {
	struct Frame {
		int16_t RH;
		int16_t T;
		uint8_t checksum;
	} frame;
	uint8_t arr[sizeof(Frame)];
} AM2302;

sysConfig EEMEM eConf = { 0xFF, 100, 300, 40, 50, 0 };
uint8_t iobuf[MAX(sizeof(sysConfig), sizeof(sysStatus))];

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

int16_t FanLevel(int16_t dT, int16_t dRH)
{
	int16_t A = (FanMax - FanMin) * dT / validConf.dDeltaT;
	int16_t B = (FanMax - FanMin) * dRH / validConf.dDeltaRH;
	return A > B ? A : B;	
}

void FanRegulation()
{
	//Upper bounds for regulation
	if (validConf.fanLevelOverride != 0xFF)
		return;
	int16_t dT = tmpStatus.insideT - tmpStatus.outsideT - validConf.minDeltaT;
	int16_t dRH = tmpStatus.insideRH - tmpStatus.outsideRH - validConf.minDeltaRH;
	int16_t A = FanLevel(dT, dRH) + FanMin;
	if ((fanLvl >= FanMin && A > fanLvl) || (fanLvl < FanMin && A > FanMin)) //At least one of upper bounds is above
		fanLvl = A > FanMax ? FanMax : A; //Increase level
	else
	{
		//Lower bounds for regulation
		dT -= validConf.minDeltaT >> 3;
		dRH -= validConf.minDeltaRH >> 3;
		A = FanLevel(dT, dRH) + FanMin;
		if (A < fanLvl) //Both lower bounds are below
			fanLvl = A > FanMin ? A : 0; //Decrease level
	}
}

uint16_t GetRPM()
{
	while (tachOvf--)
		tachSum += 0xffff;
	tachSum -= tachPrev;
	uint32_t temp = (F_CPU * 60 / 8) * (tachCnt - 1);
	return (uint16_t)(temp / tachSum);
}

ISR (USART0_RX_vect)
{
	static char uCnt;
	static uint8_t *rxBuf;
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
				rxBuf = iobuf; //First byte address in structure
				rxMark = 0;
			}
			else
			{
				U0TXen();
				if (rxMode == GetConfig)
				{
					txCnt = sizeof(sysConfig) - 1; //Because one byte will be send right after
					txBuf = (uint8_t*)&validConf;
				}
				else //Get status
				{
					txBuf = iobuf;
					tmpStatus.CRC16 = CalculateCRC16(&tmpStatus, sizeof(sysStatus) - 2);
					memcpy(iobuf, &tmpStatus, sizeof(sysStatus));
					txCnt = sizeof(sysStatus) - 1;
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
		if (!--uCnt) //Packet received
		{
			rxMode = 0;
			if (CalculateCRC16(iobuf, sizeof(sysConfig) - 2) == ((sysConfig*)iobuf)->CRC16) //CRC OK
			{
				memcpy(&validConf, iobuf, sizeof(sysConfig));
				if (validConf.fanLevelOverride != 0xFF)
				{
					fanLvl = validConf.fanLevelOverride;
					if (fanLvl < FanMin)
						fanLvl = 0;
					else if (fanLvl > FanMax)
						fanLvl = FanMax;
				}
			}
			UCSR0A = (1 << MPCM0); //Set MPCM bit
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

ISR (TIMER1_OVF_vect) //Occurs every 71.1ms
{
	uint8_t lcycle = ((uint8_t)cycles++) & 0x3F; //Range 0-63
	if (!cycles) //~77 minutes between updates
		eeprom_update_block(&validConf, &eConf, sizeof(sysConfig));
	if (lcycle == 44 || lcycle == 52) //Send start signal to the sensor
	{
		uint8_t delayz = 0, j;
		for (j = 0; j < 4; j++)
			delayz += AM2302.arr[j];
		j = AM2302.arr[1]; //Big endian to little endian conversion
		AM2302.arr[1] = AM2302.arr[0];
		AM2302.arr[0] = j;
		j = AM2302.arr[3];
		AM2302.arr[3] = AM2302.arr[2];
		AM2302.arr[2] = j;
		if (delayz == AM2302.arr[4] && AM2302.frame.RH != 0)
		{
			if (AM2302.frame.T < 0)
				AM2302.frame.T = ~(AM2302.frame.T & 0x7FFF) + 1;
			if (lcycle == 44)
			{
				tmpStatus.insideRH = AM2302.frame.RH;
				tmpStatus.insideT = AM2302.frame.T;
			}
			else
			{
				tmpStatus.outsideRH = AM2302.frame.RH;
				tmpStatus.outsideT = AM2302.frame.T;
				FanRegulation();
			}
		}

		PORTB &= ~(1 << PORTB2);
		DDRB |= (1 << DDB2); //Interface pulled low
		TCCR0B = (1 << CS02); //28.8kHz, 34.72µs tick
		TCNT0 = 0;
		OCR0B = 48; //~1.666ms delay
		TIFR0 = (1 << TOV0) | (1 << OCF0B);
		TIMSK0 = (1 << OCIE0B); //Enable interrupt
		amCnt = -2; //Skip response signal
	}
	lcycle &= 0xF; //Range 0-15
	if (!lcycle)
	{
		tmpStatus.rpmFront = tachCnt > 1 ? GetRPM() : 0;
		SelChA();
		tachCnt = 0;
		TIMSK1 |= (1 << ICIE1);
	}
	else if (lcycle == 8)
	{
		tmpStatus.rpmRear = tachCnt > 1 ? GetRPM() : 0;
		SelChB();
		tachCnt = 0;
		TIMSK1 |= (1 << ICIE1);
		tmpStatus.currentDraw = (adcSum << 3) / Idiv_x1mA;
		adcSum = 0;
	}

	if (rxMode == SetConfig && 2 < ++rxMark) //We are currently receiving data packet
	{
		UCSR0A = (1 << MPCM0);
		rxMode = 0;
	}

	if (lcycle & 1)
		ADCSRA |= (1 << ADSC);
	if ((lcycle & 3) == 3)
	{
		if (OCR2AL > fanLvl)
		{
			OCR2AL--;
			if (OCR2AL < FanMin)
				OCR2AL = 0;
		}
		else if (OCR2AL < fanLvl)
		{
			OCR2AL++;
			if (OCR2AL < FanMin)
				OCR2AL = FanMin;
		}
		tmpStatus.fanLevel = OCR2AL;
		OCR2BL = ICR2L - OCR2AL;
	}
}

ISR	(TIMER0_COMPB_vect)
{
	DDRB &= ~(1 << DDB2); //Interface released
	TCCR0B = (1 << CS01); //921.6kHz clock, 1 tick = 1.08µs
	TCNT0 = 0;
	TIMSK0 = (1 << TOIE0); //Now enable overflow interrupt, timeout 278µs
	PCMSK1 = (1 << PCINT10); //PINB2 as pin change interrupt source
	GIFR = (1 << PCIF1);
	GIMSK = (1 << PCIE1); //Enable pin change interrupt
}

ISR (TIMER0_OVF_vect)
{
	TIMSK0 = 0; //Disable overflow interrupt
	GIMSK = 0; //Disable pin change interrupt
	memset(&AM2302, 0, sizeof(am2302));
}

ISR (PCINT1_vect)
{
	static uint8_t temp;
	uint8_t delayz = TCNT0;
	TCNT0 = 0; //Clear counter
	if (PINB & (1 << PINB2))
		return; //Skip rising edge interrupt
	if (0 <= amCnt && amCnt < 40)
	{
		temp <<= 1;
		if (delayz > 46) //High level held more than 48µs - logic one received
			temp |= 1;
		if ((amCnt & 7) == 7)
			AM2302.arr[amCnt >> 3] = temp;
	}
	if (++amCnt == 40)
	{
		TIMSK0 = 0; //Disable overflow interrupt
		GIMSK = 0; //Disable pin change interrupt
	}
}

ISR (TIMER1_CAPT_vect)
{
	if (!tachCnt++)
		tachPrev = ICR1;
	else
		tachSum = ICR1;
	if (tachCnt > 7) //8 points is enough
		TIMSK1 &= ~(1 << ICIE1);
	tachOvf = (char)cycles & 7;
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
	UBRR0 = 5; //Actual maximum transfer rate: 6400Bps
	UCSR0B = (1 << RXCIE0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << TXEN0) | (1 << UCSZ02);
	UCSR0C = (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0A = (1 << MPCM0);
	//Timer 1: 921.6kHz clock, input capture on leading edge, noise filtering, OVF interrupt
	TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11);
	TIMSK1 = (1 << TOIE1);
	//Timer 2: 7.3728MHz clock, phase and frequency correct PWM, top in ICR2
	ICR2 = PWM_max; //25kHz PWM frequency
	OCR2A = 0; //Non-inverting output: off
	OCR2B = PWM_max; //Inverting output: off
	TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << COM2B0);
	TCCR2B = (1 << WGM23) | (1 << CS20);
	TOCPMSA0 |= (1 << TOCC2S1) | (1 << TOCC3S1); //OC2A non-inverting (TOCC3), OC2B inverting (TOCC2)
	TOCPMCOE |= (1 << TOCC2OE) | (1 << TOCC3OE);
	//ADC: 1.1V reference, 230.4kHz clock, ADC0 input, interrupt
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