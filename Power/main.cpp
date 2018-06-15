/* Power.cpp
 * Created: 25.01.2018 17:00:54
 * Version: 1.0 */

#include "PC.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>

int16_t adcResult[2] = { 0, 0 };
int8_t adcCnt = -1;
uint8_t toffCnt = 0, iclCnt = 6;

ISR(ADC_vect)
{
	adcResult[adcCnt & 1] += ADC;
}

ISR(TIM0_COMPA_vect)
{
	if (!Is_Battery_discharged && !We_Are_OffLine)
	{
		ICL_Active;
		DC_to_Load;
		_delay_ms(5); //Release time of ICL
		DC_MOS_On;
		iclCnt = 6;
		Set_ChargeReq_flag;
	}
	Set_Tick_flag;
	toffCnt = 0;
}

ISR(ANA_COMP_vect) //Comparator toggle
{
	TCNT0 = 0; //Clear timer 0
	if (ACSR & (1 << ACO)) //Rising edge
	{
		Set_Tick_flag;
		if (toffCnt < 250) //~2.5s delay
			toffCnt++;
		else
		{
			ICL_Active;
			_delay_ms(5);
			DC_MOS_Off;
			_delay_ms(5);
			AC_to_Load;
			iclCnt = 6;
		}
	}
}

inline void mcuInit()
{
	cli();
	//Clock: 4MHz
	clock_prescale_set(clock_div_2);
	//Port A: pin 1 - BMS select, pin 4 - DC switch, pin 5 - cooler PWM, pin 6 - HB LED, pin 7 - main switch
	DDRA = (1 << DDA1) | (1 << DDA4) | (1 << DDA5) | (1 << DDA6) | (1 << DDA7);
	//Port B: pin 0 - charger switch, pin 1 - cooler switch, pin 2 - ICL switch
	DDRB = (1 << DDB0) | (1 << DDB1) | (1 << DDB2);
	//Timer 0: 3906Hz, CTC, interrupt on OC0A
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS02) | (1 << CS00);
	OCR0A = 40; //10.24ms timeout
	TIMSK0 = (1 << OCIE0A);
	//Timer 1: 4MHz, fast PWM with top in ICR1
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
	TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);
	ICR1 = PWM_max;
	OCR1A = 0; //HB LED is off
	OCR1B = 0; //Cooler is off
	//Analog comparator: 1.1v on AIN+, interrupt
	ACSR = (1 << ACBG) | (1 << ACIE);
	//Digital input disable: THS0, AC detection, battery sense
	DIDR0 = (1 << ADC0D) | (1 << ADC2D) | (1 << ADC3D);
	//ADC: 3.3v reference, 125kHz, interrupt
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS0);
	//Power reduction: USI
	PRR = (1 << PRUSI);
	sei();
}

int main(void)
{
	uint32_t chrgCnt = 5;
	int32_t k;
	int16_t m, cmax = 127;
	uint8_t sysTick = 0;
	int8_t hb = 0;
	mcuInit();
    /* Replace with your application code */
    while (true)
	{
		while (!Is_Tick_set);
		sysTick++; //~10ms period
		if (iclCnt > 0)
		{
			if (--iclCnt == 0)
				ICL_Shunted;
		}

		if (!(sysTick & 7)) //~80ms period
		{
			//Heartbeat LED
			m = hb++;
			m = (m * m) >> 8;
			OCR1A = m > cmax ? cmax : m;
			if (++adcCnt > 7) //Each channel has been sampled four times
			{
				//Top level for HB LED
				m = adcResult[0] - TSEN_ZERO;
				k = m * 128;
				k /= TSEN_MIN - TSEN_ZERO; //0…60°C range
				cmax = (int16_t)k > 127 ? 127 : k;
				//Cooler regulation
				adcResult[0] -= TSEN_MIN;
				k = adcResult[0] * (FanMax - FanMin);
				k /= TSEN_MAX - TSEN_MIN; //Upper bound of regulation
				m = (int16_t)k + FanMin;
				if (m < 0)
					m = 0;
				if ((uint16_t)m > OCR1B)
					OCR1B = m > FanMax ? FanMax : m;
				else
				{
					adcResult[0] -= (TSEN_MAX - TSEN_MIN) >> 3; //Add 12.5% hysteresis
					k = adcResult[0] * (FanMax - FanMin);
					k /= TSEN_MAX - TSEN_MIN; //Lower bound of regulation
					m = (int16_t)k + FanMin;
					if (m < 0)
						m = 0;
					if ((uint16_t)m < OCR1B)
						OCR1B = m < FanMin ? 0 : m;
				}
				if (OCR1B)
					Cooler_On;
				else
					Cooler_Off;

				if (adcResult[1] < DC_MIN && We_Are_OffLine) //Check battery voltage
				{
					Set_Discharged_flag;
					ICL_Active;
					_delay_ms(5); //Maximum relay release time
					DC_MOS_Off;
				}

				if (!(PORTA & (1 << PORTA4))) //Charger is off
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
					else if (BMS_B_Selected) //Happens when BMS 2 finished charge cycle
						BMS_Sel_A;
				}
				else if (!We_Are_OffLine)
				{
					if (adcResult[1] < (DC_MIN >> 1)) //To detect continuous low level
						chrgCnt--;
					if (chrgCnt == 0) //Charge for selected BMS finished
					{
						Charger_Off;
						if (BMS_B_Selected)
						{
							chrgCnt = 135000; //1.5 days delay
							Clear_Discharged_flag;
						}
						else
							chrgCnt = 5;
					}
				}
				adcCnt = 0;
				adcResult[0] = adcResult[1] = 0;
			}

			if (Is_Charge_Required)
			{
				Clear_ChargeReq_flag;
				chrgCnt = 6; //Set minimum delay
			}
			//ADC conversion routine
			if (adcCnt & 1)
				ADMUX = 0; //ADC0 - THS0
			else
				ADMUX = (1 << MUX0) | (1 << MUX1); //ADC3 - battery voltage
			ADCSRA |= (1 << ADSC); //Start conversion
		}

		Clear_Tick_flag;
	}
}