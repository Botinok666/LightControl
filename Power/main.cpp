/* Power.cpp
 * Created: 25.01.2018 17:00:54
 * Version: 2.0 */

#include "PC.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "twi.h"
#include "lcdtwi.h"
#include <stdint-gcc.h>

volatile int16_t adcResult = 0, currentPWM = 24;
volatile uint16_t toffCnt = 0;
volatile bool TickFlag = false, Discharged = false, ChargeReq = true;
uint8_t captOvfCtr = 0, lastOvf = 0;
uint16_t captFirst, captCtr = 0;
uint32_t captSum;
uint8_t buffer[] = {
	0x74,0x20,0x20,0x20,0x2F,0x20,0x20,0x00,0x43,0x20,0x01,0x20,0x20,0x20,0x25,0x20, //t 25/25°C  100%
	0x20,0x20,0x20,0x20,0x20,0x20,0x72,0x70,0x6D,0x20,0x20,0x20,0x20,0x20,0x57,0x20  // 1002 rpm  118W
};
const uint8_t custom_chars[] = {
	6,9,9,6,0,0,0,0,
	14,31,19,21,21,25,31,0
};

void uint2buffer(uint8_t k_index, uint16_t k_value)
{
	uint8_t stx, sti, hide_zero = 0, k_data;
	uint16_t divider = 1000;
	for (stx = 0; stx < 4; stx++)
	{
		sti = k_value / divider;
		k_value -= sti * divider;
		if (!sti && !hide_zero && stx < 3)
		k_data = 0x20;
		else
		{
			hide_zero = 1;
			k_data = sti + 0x30;
		}
		divider /= 10;
		buffer[k_index++] = k_data;
	}
}

ISR (ADC_vect)
{
	adcResult += ADC;
}

ISR (TIMER1_CAPT_vect)
{
	static uint8_t ovfFirst = 0;
	if (!captCtr++)
	{
		captFirst = ICR1;
		ovfFirst = captOvfCtr;
	}
	else
		captSum = ICR1;
	lastOvf = captOvfCtr - ovfFirst;
}

ISR (TIMER1_OVF_vect)
{
	captOvfCtr++;
	if (OCR0B < currentPWM)
		OCR0B++;
	else if (OCR0B > currentPWM)
		OCR0B--;

	if (captOvfCtr >= 16) //~1s period
	{
		if (captCtr > 1)
		{
			captSum += (uint32_t)lastOvf << 16;
			captSum -= captFirst;
			captSum /= 10;
			uint32_t temp = (F_CPU * 3 / 8UL) * (captCtr - 1);
			uint2buffer(17, (uint16_t)(temp / captSum));
		}
		else
			uint2buffer(17, 0);
		captOvfCtr = 0;
		TickFlag = true;
		captCtr = 0;
		adcResult >>= 4;
	}
	//ADC conversion routine
	ADCSRA |= (1 << ADSC); //Start conversion
}

ISR (TIMER2_COMPA_vect)
{
	if (!Discharged && !We_Are_OffLine)
	{
		AC_MOS_Off;
		Charger_Off;
		_delay_us(66); //Dead time
		DC_MOS_On;
		ChargeReq = true;
		TCNT2 = 0;
	}
	toffCnt = 0;

	TIFR2 = (1 << OCF2B) | (1 << OCF2A);
	TIMSK2 = (1 << OCIE2B); //Disable timeout and enable 10ms interrupt
}

ISR (TIMER2_COMPB_vect)
{
	TCNT2 = 0;
}

ISR (INT0_vect) //Mains voltage sense
{
	static bool trig = false;
	if (PORTD & (1 << PORTD2)) //Rising edge: mains voltage below threshold
	{
		if (trig)
		{
			TCNT2 = 0; //Clear timer 2
			_delay_us(560); //At 230VAC provides delay to hit zero crossing
			AC_MOS_On;
			trig = false;
		}
		TIMSK2 = (1 << OCIE2A); //Disable 10ms and enable timeout interrupt
	}
	else //Falling edge: mains voltage above threshold
	{
		if (toffCnt < 500) //~5s delay
			toffCnt++;
		else if (!(PORTD & ((1 << PORTD0) | (1 << PORTD1)))) //Both MOS switches are off
			trig = true;
		else if (!(PORTD & (1 << PORTD0))) //DC MOS should be turned off first
			ACSR = (1 << ACBG) | (1 << ACIE); //Enable toggle interrupt
		TIMSK2 = 0; //Disable timeout and 10ms interrupts
	}
	TCNT2 = 0; //Clear timer 2
	TIFR2 = (1 << OCF2B) | (1 << OCF2A);
}

ISR (ANALOG_COMP_vect)
{
	ACSR = (1 << ACBG); //Disable interrupt
	DC_MOS_Off;
	_delay_us(66); //Dead time
	AC_MOS_On;
}

inline void mcuInit()
{
	//Port B: pin 1 - charger on, pin 2 - charger select
	DDRB = (1 << DDB1) | (1 << DDB2);
	//Port C: pin 5 - SCL
	DDRC = (1 << DDC5);
	//Port D: 0 - mains switch, 1 - DC switch, 5 - cooler PWM
	DDRD = (1 << DDD0) | (1 << DDD1) | (1 << DDD5);
	//External interrupts: INT0 enabled
	EICRA = (1 << ISC00); //Any logical change generates an interrupt
	EIMSK = (1 << INT0);
	//Timer 0: 8MHz, phase and frequency correct PWM with top in OCR0A
	TCCR0A = (1 << COM0B1) | (1 << WGM00);
	TCCR0B = (1 << WGM02) | (1 << CS00);
	OCR0A = FanMax; //~25kHz
	OCR0B = 64; //40% DC at power-on
	//Timer 1: 1MHz, input capture and overflow interrupt
	TCCR1B = (1 << ICNC1) | (1 << CS11); //~15 overflows/s
	TIMSK1 = (1 << TOIE1) | (1 << ICIE1);
	//Timer 2: 7812Hz, interrupt on OC2A
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
	OCR2A = 13; //1.79ms timeout: ~140VAC minimum input voltage
	OCR2B = 77; //10ms period
	//Analog comparator: 1.1v on AIN0
	ACSR = (1 << ACBG);
	//ADC: 1.1v reference, ADC6 input, 125kHz, interrupt
	ADMUX = (1 << REFS0) | (1 << REFS1) | (1 << MUX2) | (1 << MUX1);
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);
	//TWI: 25kHz
	TWBR = 117; //1 byte transfer time ~250µs
	//Power reduction: SPI and USART
	PRR = (1 << PRSPI) | (1 << PRUSART0);
	sei();
}

int main(void)
{
	uint32_t chrgCnt = 5;
	int32_t k;
	int16_t m, ths1, ths2;
	mcuInit();

	LCDInit();
	LCDCmd(0x40);
	LCDBuffer(custom_chars, 16);

	twi_start();
	twi_send(THS1_TWI_WR);
	twi_send(0x00); //Temperature register
	twi_stop();
	twi_start();
	twi_send(THS2_TWI_WR);
	twi_send(0x00); //Temperature register
	twi_stop();
	twi_start();
	twi_send(IOUT_TWI_WR);
	twi_send(0x00); //Configuration
	twi_send(0x31); //MSB: gain /4
	twi_send(0xFD); //LSB: 128 samples, shunt voltage continuous sampling
	twi_stop();
    /* Replace with your application code */
    while (true)
	{
		while (!TickFlag); //~1s period
		twi_start();
		twi_send(THS1_TWI_RD);
		ths1 = (int16_t)twi_receive(1) << 2;
		ths1 |= twi_receive(0) >> 6;
		twi_stop();
		uint2buffer(3, (ths1 + 2) >> 2);

		twi_start();
		twi_send(THS2_TWI_RD);
		ths2 = (int16_t)twi_receive(1) << 2;
		ths2 |= twi_receive(0) >> 6;
		twi_stop();
		uint2buffer(0, (ths2 + 2) >> 2);
		buffer[0] = 0x74;
		buffer[4] = 0x2F;

		twi_start();
		twi_send(IOUT_TWI_WR);
		twi_send(0x01); //Shunt voltage
		twi_start();
		twi_send(IOUT_TWI_RD);
		k = (int16_t)twi_receive(1) << 8;
		k |= twi_receive(0);
		twi_stop();
		k *= 222; //V_out
		k /= 2500; //R_sense * 100, result: power in watts
		uint2buffer(26, k);

		k *= Bat_ESR;
		adcResult <<= 1;
		if (adcResult > 0) //Prevent divide by zero
			adcResult += k / adcResult; //Corrected battery voltage (now without ESR)
		if (adcResult > Bat_12p)
			m = 12 + (adcResult - Bat_12p) / Charge_HDiv;
		else
			m = (adcResult - Bat_1p) / Charge_LDiv;
		uint2buffer(10, m > 100 ? 100 : m < 0 ? 0 : m);
		buffer[10] = 0x01;

		if (m < 1 && We_Are_OffLine) //Check battery charge level
		{
			Discharged = true;
			DC_MOS_Off;
		}

		if (ths2 > ths1)
			ths1 = ths2;
		//Cooler regulation
		k = ths1 - TSEN_MIN; //dT
		k *= FanMax - FanMin;
		k /= TSEN_MAX - TSEN_MIN; //Upper bound of regulation
		m = (int16_t)k + FanMin; //A
		if (m > currentPWM)
			currentPWM = m > FanMax ? FanMax : m;
		else
		{
			k = ths1 - TSEN_MIN + ((TSEN_MAX - TSEN_MIN) >> 3); //Add 12.5% hysteresis
			k *= FanMax - FanMin;
			k /= TSEN_MAX - TSEN_MIN; //Lower bound of regulation
			m = (int16_t)k + FanMin; //A
			if (m < currentPWM)
				currentPWM = m < FanMin ? FanMin : m;
		}

		if (!(PORTB & (1 << PORTB1))) //Charger is off
		{
			if (!We_Are_OffLine)
				chrgCnt--;
			if (chrgCnt == 1)
			{
				if (!BMS_B_Selected)
					BMS_Sel_B;
			}
			else if (chrgCnt == 0)
			{
				Charger_On; //Power on the charger
				chrgCnt = 6;
			}
			else if (BMS_B_Selected) //Happens when BMS 2 finishes charge cycle
				BMS_Sel_A;
		}
		else if (!We_Are_OffLine)
		{
			if (!(PORTD & (1 << PORTD4))) //To detect continuous low level
				chrgCnt--;
			if (chrgCnt == 0) //Charge for the selected BMS finished
			{
				Charger_Off;
				if (BMS_B_Selected)
				{
					chrgCnt = 135000; //1.5 days delay
					Discharged = false;
				}
				else
					chrgCnt = 5;
			}
		}
			
		if (ChargeReq)
		{
			ChargeReq = false;
			chrgCnt = 6; //Set minimum delay
		}
		adcResult = 0;
		LCDCmd(0x80);
		LCDBuffer(buffer, 16);
		LCDCmd(0xC0);
		LCDBuffer(buffer + 16, 16);
		TickFlag = false;
	}
}
