/* Power.cpp
 * Created: 25.01.2018 17:00:54
 * Version: 1.0 */

#include "PC.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>

int16_t adcResult[2] = { 0, 0 };
volatile int8_t adcCnt = -1;
volatile uint8_t iclCnt = 6;
volatile uint16_t toffCnt = 0;
volatile bool TickFlag = false, Discharged = false, ChargeReq = true;

ISR (ADC_vect)
{
	adcResult[adcCnt & 1] += ADC;
}

ISR (TIM0_COMPA_vect)
{
	if (!Discharged && !We_Are_OffLine)
	{
		ICL_Active;
		_delay_ms(5); //Release time of ICL relay
		DC_to_Load;
		DC_MOS_On;
		iclCnt = 6;
		ChargeReq = true;
		TCNT0 = 0;
		ACSR = (1 << ACBG) | (1 << ACIE) | (1 << ACI); //Prevent immediate firing of ACMP interrupt
	}
	toffCnt = 0;

	TIFR0 = (1 << OCF0B) | (1 << OCF0A);
	TIMSK0 = (1 << OCIE0B); //Disable timeout and enable 10ms interrupt
}

ISR (TIM0_COMPB_vect)
{
	TCNT0 = 0;
	TickFlag = true;
}

ISR (ANA_COMP_vect) //Comparator toggle
{
	if (ACSR & (1 << ACO)) //Rising edge: mains voltage below threshold
		TIMSK0 = (1 << OCIE0A); //Disable 10ms and enable timeout interrupt
	else //Falling edge: mains voltage above threshold
	{
		TickFlag = true;
		if (toffCnt < 500) //~5s delay
			toffCnt++;
		else
		{
			ICL_Active;
			_delay_ms(5); //Release time of ICL relay
			DC_MOS_Off;
			AC_to_Load;
			iclCnt = 6;
			ACSR = (1 << ACBG) | (1 << ACIE) | (1 << ACI); //Prevent immediate firing of ACMP interrupt
			OCR1BL = (FanMax + FanMin) >> 1;
		}
		TIMSK0 = 0; //Disable timeout and 10ms interrupts
	}
	TCNT0 = 0; //Clear timer 0
	TIFR0 = (1 << OCF0B) | (1 << OCF0A);
}

inline void mcuInit()
{
	cli();
	//Clock: 8MHz
	clock_prescale_set(clock_div_1);
	//Port A: pin 1 - BMS select, pin 4 - DC switch, pin 5 - cooler PWM, pin 6 - HB LED, pin 7 - main switch
	DDRA = (1 << DDA1) | (1 << DDA4) | (1 << DDA5) | (1 << DDA6) | (1 << DDA7);
	//Port B: pin 0 - charger switch, pin 2 - ICL switch
	DDRB = (1 << DDB0) | (1 << DDB2);
	//Timer 0: 7812Hz, interrupt on OC0A
	TCCR0B = (1 << CS02) | (1 << CS00);
	OCR0A = 13; //1.79ms timeout: ~140VAC minimum input voltage
	OCR0B = 77; //10ms period
	//Timer 1: 8MHz, phase and frequency correct PWM with top in ICR1
	TCCR1A = (1 << COM1A1) | (1 << COM1B1);
	TCCR1B = (1 << WGM13) | (1 << CS10);
	ICR1 = PWM_max;
	OCR1A = 0; //HB LED is off
	OCR1B = 0; //Cooler is off
	//Analog comparator: 1.1v on AIN0, interrupt on toggle
	ACSR = (1 << ACBG) | (1 << ACIE);
	//Digital input disable: THS0, AC detection, battery sense
	DIDR0 = (1 << ADC0D) | (1 << ADC2D) | (1 << ADC3D);
	//ADC: 3.3v reference, 125kHz, interrupt
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);
	//Power reduction: USI
	PRR = (1 << PRUSI);
	sei();
}

int main(void)
{
	uint32_t chrgCnt = 5;
	int32_t k;
	int16_t m;
	uint8_t sysTick = 0, fanLvl = FanMin;
	int8_t hb;
	mcuInit();
    /* Replace with your application code */
    while (true)
	{
		while (!TickFlag);
		sysTick++; //~10ms period
		if (iclCnt > 0)
		{
			if (--iclCnt == 0)
				ICL_Shunted;
		}

		if (!(sysTick & 7)) //~80ms period
		{
			//Heartbeat LED
			//m = hb++;
			//OCR1A = (m * m) >> 8;
			if (++adcCnt > 7) //Each channel has been sampled four times
			{
				//Output level for LED for testing purpose
				m = adcResult[0] - TSEN_ZERO;
				k = m * PWM_max;
				k /= TSEN_MIN - TSEN_ZERO; //0…50°C range
				OCR1A = (int16_t)k > PWM_max ? PWM_max : k;
				//Cooler regulation
				adcResult[0] -= TSEN_MIN; //dT
				k = adcResult[0] * (FanMax - FanMin);
				k /= TSEN_MAX - TSEN_MIN; //Upper bound of regulation
				m = (int16_t)k + FanMin; //A
				if (m > fanLvl)
					fanLvl = m > FanMax ? FanMax : m;
				else
				{
					adcResult[0] += (TSEN_MAX - TSEN_MIN) >> 3; //Add 12.5% hysteresis
					k = adcResult[0] * (FanMax - FanMin);
					k /= TSEN_MAX - TSEN_MIN; //Lower bound of regulation
					m = (int16_t)k + FanMin; //A
					if (m < fanLvl)
						fanLvl = m < FanMin ? FanMin : m;
				}
				if (OCR1BL > fanLvl)
					OCR1BL--;
				else if (OCR1BL < fanLvl)
					OCR1BL++;

				if (adcResult[1] < DC_MIN && We_Are_OffLine) //Check battery voltage
				{
					Discharged = true;
					ICL_Active;
					_delay_ms(5); //ICL relay release time
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
					else if (BMS_B_Selected) //Happens when BMS 2 finishes charge cycle
						BMS_Sel_A;
				}
				else if (!We_Are_OffLine)
				{
					if (adcResult[1] < (DC_MIN >> 1)) //To detect continuous low level
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
				adcCnt = 0;
				adcResult[0] = adcResult[1] = 0;
			}

			if (ChargeReq)
			{
				ChargeReq = false;
				chrgCnt = 6; //Set minimum delay
			}
			//ADC conversion routine
			if (adcCnt & 1)
				ADMUX = 0; //ADC0 - THS0
			else
				ADMUX = (1 << MUX0) | (1 << MUX1); //ADC3 - battery voltage
			ADCSRA |= (1 << ADSC); //Start conversion
		}

		TickFlag = false;
	}
}