/* DCUPS.c
 * Created: 7/2/2019 23:30:30
 * Author : Andrew */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>
#include "main.h"
#include "twi.h"
#include "lcdtwi.h"
#include <util/delay.h>

//#define TEST_VI
//#define TEST_VB

volatile uint8_t captOvfCtr = 0, lastOvf = 0, fanLvl = 0, safeWait = 0, acTrState = 0, slaIdxRam = 20;
volatile uint16_t captFirst, captCtr = 0, captCS, timeout = 333, chargeRestartCounter = 0;
volatile uint32_t captSum, captSS;

uint8_t EEMEM slaFailIdx = 20;

const uint8_t accuVSSel[20] = { 
	0x30,0x31,0x32,0x37,0x36,0x34,0x28,0x29,0x2A,0x2D,
	0x2F,0x2E,0x2C,0x18,0x19,0x1A,0x1D,0x1F,0x1E,0x1C };
const uint16_t vSenDiv[20] = {
	3037,3043,3037,3040,3066,3043,3044,3048,3042,3048,
	3061,3050,3041,3055,3051,3053,3054,3060,3061,3054 };
uint16_t accuVS[20], accuVSValid[20];

//Placement of the information on the LCD
//0  3   7   B   F
//000.0v      00�C| <- test version
// 0.000A 0000 rpm|
//000W A13.7v 00�C| <- normal version
//T B 00% 0000 rpm| (2nd row on battery)
//T B P I 0000 rpm| (2nd row on line)
uint8_t back_buffer[32] = {
#ifdef TEST_VI
	0x20,0x20,0x20,0x57,0x20,0x76,0x20,0x20,0x20,0x20,0x76,0x20,0x20,0x20,0x00,0x43,
	0x20,0x20,0x20,0x20,0x20,0x20,0x41,0x20,0x20,0x20,0x20,0x20,0x20,0x72,0x70,0x6D
#else
	0x20,0x20,0x20,0x57,0x20,0x20,0x20,0x20,0x20,0x20,0x76,0x20,0x20,0x20,0x00,0x43,
	0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x72,0x70,0x6D
#endif
	};

void i2c_init(void)
{
	//Configure INA219
	twi_set_register(INA219_wr, 0); //Configuration
	twi_send(0x38);
	twi_send(0x79); //Shunt voltage one-shot, 320mV range, 128 samples averaging
	twi_stop();
	//Configure MCP3421
	twi_start();
	twi_send(MCP3421_wr);
	twi_send(0x88); //Initiate one-shot conversion
	twi_stop();
	//Load custom characters into LCD
	uint8_t custom_chars[] = {
		0b00110, //0x00
		0b01001,
		0b01001,
		0b00110,
		0b00000,
		0b00000,
		0b00000,
		0b00000,
		
		0b01110, //0x01
		0b11111,
		0b10001,
		0b10101,
		0b11111,
		0b10101,
		0b11111,
		0b00000,
		
		0b01110, //0x02
		0b11111,
		0b10001,
		0b10001,
		0b10001,
		0b10001,
		0b11111,
		0b00000,
		
		0b01010, //0x03
		0b01010,
		0b11111,
		0b10001,
		0b10001,
		0b01110,
		0b00100,
		0b00000 };
	lcd_send_byte(0x40, 0);
	lcd_send_buffer(custom_chars, sizeof(custom_chars));
}

void uint2buffer(uint8_t *buffer_index, uint8_t dot_position, uint8_t length, uint16_t value) {
	uint8_t stx, sti, hide_zero = 0, k_data;
	uint16_t divider = 1;
	sti = length - 1;
	if (dot_position < length)
		sti--;
	for (stx = 0; stx < sti; stx++)
		divider *= 10;
	for (stx = 0; stx < length; stx++)
	{
		if (stx != dot_position)
		{
			sti = value / divider;
			value -= sti * divider;
			if ((stx < dot_position) && (sti == 0) && (hide_zero == 0) && (divider > 1))
				k_data = 0x20;
			else
			{
				hide_zero = 1;
				k_data = sti + 0x30;
			}
			divider /= 10;
		}
		else
			k_data = 0x2E;
		*buffer_index++ = k_data;
	}
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
	TIFR1 = (1 << ICF1);
}

ISR (TIMER1_OVF_vect)
{
	uint8_t temp = OCR0A;
	if (temp < fanLvl) {
		if (temp < FAN_min) {
			PORTD |= (1 << PORTD5); //Turn on fan power
			temp = FAN_min;
		}
		else
			temp++;
	}
	else if (temp > fanLvl) {
		if (fanLvl < FAN_min) {
			PORTD &= ~(1 << PORTD5); //Turn off fan power
			temp = 0;
		}
		else
			temp--;
	}
	OCR0A = temp;
	safeWait++;
	if (captOvfCtr++ >= 15) //~1s period
	{
		captOvfCtr = 0;
		captCS = captCtr;
		captCtr = 0;
		captSum += (uint32_t)lastOvf << 16;
		captSS = captSum - captFirst;
		
		chargeRestartCounter++;
	}
}

ISR (INT0_vect) //AC detect
{
	static uint8_t lastHTL = 15;
	
	if (PIND & (1 << PIND2)) //Low-to-high change -> AC voltage below threshold
	{
		TCNT2 = 0; //Reset timer 2
		TIFR2 = (1 << OCF2A) | (1 << OCF2B); //Clear pending interrupts from timer 2
		if (WE_ARE_OFFLINE)
		{
			if (timeout > 0)
				timeout--;
			else
			{
				DC_IGBT_off;
				_delay_us(66);
				DC_MOS_off;
				while (TCNT2 < (lastHTL >> 1)); //Wait until zero cross
				AC_IGBT_on;
				acTrState = 1; //Restart charging cycle
			}
		}
		TIMSK2 = (1 << OCIE2A); //Enable interrupt A
	}
	else //High-to-low transition -> AC voltage above threshold
	{
		lastHTL = TCNT2; //Remember time spent with AC below threshold
		TCNT2 = 0; //Delay 9.5ms
		TIFR2 = (1 << OCF2A) | (1 << OCF2B); //Clear pending interrupts from timer 2
		TIMSK2 = (1 << OCIE2B); //Enable interrupt B
	}
}

ISR (TIMER2_COMPA_vect) //AC loss detect
{
	if (WE_ARE_ONLINE)
	{
		AC_IGBT_off;
		_delay_us(66);
		DC_MOS_on;
		TIFR2 = (1 << OCF2A) | (1 << OCF2B); 
		TIMSK2 = (1 << OCIE2B);
		slaIdxRam = 20;
		back_buffer[16] = 0x20;
		ALERT_off;
	}
	timeout = 666; //6.66s
}

ISR (TIMER2_COMPB_vect) //10ms period
{
	if (WE_ARE_OFFLINE)
	{
		if (ONLY_ICL_IS_ON)
			DC_IGBT_on;
		TIMSK2 = 0; //Disable interrupts from timer 2
	}
	else //AC loss detect
	{
		AC_IGBT_off;
		_delay_us(228);
		DC_MOS_on;
		TCNT2 = 0;
		TIFR2 = (1 << OCF2A) | (1 << OCF2B);
		TIMSK2 = (1 << OCIE2B);
		slaIdxRam = 20;
		back_buffer[16] = 0x20;
		ALERT_off;
	}
	timeout = 666; //6.66s
}

void inline mcu_init(void)
{
	//Port B: 0 - fan tachometer, 1 - alert on (out)
	DDRB = (1 << DDB1);
	//Port C: 1 - DC MOS (out), 2 - DC IGBT (out), 3 - AC IGBT (out), 4 - SDA, 5 - SCL
	DDRC = (1 << DDC1) | (1 << DDC2) | (1 << DDC3);
	//Port D: 2 - AC detect, 5 - fan on (out), 6 - fan PWM (out), 7 - charge end detect
	DDRD = (1 << DDD5) | (1 << DDD6);
	//Interrupts: INT0 - AC detect, both edges
	EICRA = (1 << ISC00);
	EIMSK = (1 << INT0);
	//Timer 0: 8MHz, OC0A fast PWM 31.25kHz
	TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1);
	TCCR0B = (1 << CS00);
	//Timer 1: 1MHz, input capture and overflow interrupt
	TCCR1B = (1 << ICNC1) | (1 << CS11); //~15 overflows/s
	TIMSK1 = (1 << TOIE1) | (1 << ICIE1);
	//Timer 2: 7.812kHz, 1 tick = 128�s
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
	OCR2A = 15; //Maximum t_high (Vin = 150VAC) is 1.92ms (15 ticks)
	OCR2B = 77; //10ms period
	//TWI: 20kHz
	TWBR = 12;
	TWSR = (1 << TWPS1);
}

int main(void)
{
#ifndef TEST_VI
	char isBCharged = 0, isACharged = 0;
#endif
	char bmsSelB = 0, bmsOn = 0; //CHON - bit 6, CHSEL - bit 7
	uint8_t vsenIdx = 0, dispIdx = 0, countdown = 5;
	uint16_t vSlaFail = 90;
	mcu_init();
	lcd_init();
	i2c_init();
	if (eeprom_read_byte(&slaFailIdx) < 20)
	{
		back_buffer[16] = 'A' + eeprom_read_byte(&slaFailIdx);
		ALERT_on;
	}
	sei();
	
    while (1) 
    {	
		uint8_t temp8;
		uint16_t current;
		uint16_t temp16, tDelta;
		uint32_t temp32;
		//Read temperature of the IGBT array heat sink
		twi_set_register(AD7416_wr, 0);
		twi_start();
		twi_send(AD7416_rd);
		temp16 = (uint16_t)twi_receive(1) << 8;
		temp16 |= twi_receive(0); //Contains (T * 256)
		twi_stop();
		temp16 += 0x7F;
		uint2buffer(back_buffer + 12, 2, 2, temp16 >> 8); //Rounded T
		tDelta = temp16 > T_LOWER ? temp16 - T_LOWER : 0;
		//Read current drawn from battery
		twi_set_register(INA219_wr, 1); //Shunt voltage, contains (I * 2500)
		twi_start();
		twi_send(INA219_rd);
		current = (uint16_t)twi_receive(1) << 8;
		current |= twi_receive(0);
		twi_set_register(INA219_wr, 0); //Configuration
		twi_send(0x38);
		twi_send(0x79); //Shunt voltage one-shot, 320mV range, 128 samples averaging
		twi_stop();
		
		current = (int16_t)current + 2; //Offset ~1mA
		if ((int16_t)current < 0)
			current = 0;
		temp32 = (uint32_t)current << 8;
		temp32 += 547 / 2;
		current = temp32 / 547; //K = 2.5 with gain error -1.1698
		
	#ifdef TEST_VI
		uint2buffer(back_buffer + 17, 1, 5, current); //I * 1000
	#endif
		//Read battery voltages: 7 at a time
		for (uint8_t j = 0; j < 7; j++)
		{
			//Select input channel for ADC
			twi_start();
			twi_send(PCF8574_wr);
			temp8 = accuVSSel[vsenIdx];
			if (bmsOn)
				temp8 |= (1 << 6);
			if (bmsSelB)
				temp8 |= (1 << 7);
			twi_send(temp8); //Write value to external port
			twi_stop();
			//Start ADC conversion
			twi_start();
			twi_send(MCP3421_wr);
			twi_send(0x88); //16 bit, one shot
			twi_stop();
			_delay_ms(91);
			//Read conversion data
			twi_start();
			twi_send(MCP3421_rd);
			temp16 = (uint16_t)twi_receive(1) << 8;
			temp16 |= (uint16_t)twi_receive(0);
			twi_stop();
			if ((int16_t)temp16 < 0)
				temp16 = 0;
			temp32 = (uint32_t)temp16 << 8;
			//Store battery voltage
			temp32 += vSenDiv[vsenIdx] / 2;
			temp32 /= vSenDiv[vsenIdx]; //Rounded voltage
			accuVS[vsenIdx] = temp32;
			if (++vsenIdx > 19) //All battery voltages has been read
			{
			#ifdef TEST_VI
				uint2buffer(back_buffer, 3, 5, accuVS[14]); //V * 10
			#endif
				temp8 = 20;
				if (accuVS[0] < vSlaFail)
					temp8 = 0;
				for (uint8_t k = 0; k < 19; k++)
				{
					if (accuVS[k + 1] - accuVS[k] < vSlaFail)
					{
						temp8 = k + 1;
						break;
					}
				}
				if (WE_ARE_OFFLINE && (temp8 < 20 || current > 7200))
				{
					if (current > 750) //Shutdown immediately
					{
						eeprom_write_byte(&slaFailIdx, temp8);
						DC_IGBT_off;
						_delay_us(66);
						DC_MOS_off;							
					}
					else //Low current consumption? Only alert
					{
						back_buffer[16] = 'A' + temp8;
						ALERT_on;
					}
				}
				memcpy(accuVSValid, accuVS, 20 * sizeof(uint16_t));
				vsenIdx = 0;
				break;
			}
		}
		temp32 = accuVS[19];
		temp32 *= current;
		temp32 += 5000;
		uint2buffer(back_buffer, 3, 3, temp32 / 10000); //P, rounded
		temp8 = dispIdx >> 1;
		temp16 = accuVSValid[temp8];
	#ifndef TEST_VB	
		//Show single battery voltage
		back_buffer[5] = 'A' + temp8;
		if (temp8 > 0)
			temp16 -= accuVSValid[temp8 - 1];
		uint2buffer(back_buffer + 6, 2, 4, temp16);
	#else
		uint2buffer(back_buffer + 5, 3, 5, temp16);
	#endif
		if (++dispIdx > 39)
			dispIdx = 0;
	#ifndef TEST_VI
		//If DC to AC transition succeeded
		if (acTrState)
		{
			acTrState = 0;
			back_buffer[16] = 0x20; //Clear failed battery index
			ALERT_off;
		}
		//Battery charging routine
		if (WE_ARE_ONLINE)
		{
			if (chargeRestartCounter < CHARGE_DELAY)
			{
				bmsOn = 0; //Chargers off
				bmsSelB = 0; //Relay off (group A selected)
				isACharged = isBCharged = 0; //Assume that groups are not charged
			}
			else if (chargeRestartCounter < CHARGE_CYCLE + CHARGE_DELAY)
			{
				bmsOn = 1; //Chargers on for 1 hour (after delay)
			}
			else if (chargeRestartCounter < CHARGE_CYCLE + 2 * CHARGE_DELAY)
			{
				if (bmsOn && IS_CHARGE_END) //Detect if charging of group A is ended
					isACharged = 1;
				bmsOn = 0; //Chargers off
			}
			else if (chargeRestartCounter < CHARGE_CYCLE + 3 * CHARGE_DELAY)
			{
				bmsSelB = 1; //Select group B (after delay)
			}
			else if (chargeRestartCounter < 2 * CHARGE_CYCLE + 3 * CHARGE_DELAY)
			{
				bmsOn = 1; //Chargers on for 1 hour (after delay)
			}
			else if (chargeRestartCounter < 2 * CHARGE_CYCLE + 4 * CHARGE_DELAY)
			{
				if (bmsOn && IS_CHARGE_END) //Detect if charging of group B is ended
					isBCharged = 1;
				bmsOn = 0; //Chargers off
			}
			else if (chargeRestartCounter < CHARGE_RESTART)
			{
				bmsSelB = 0; //Select group A (after delay)
				if (!isACharged || !isBCharged)
					chargeRestartCounter = 0; //If any of the groups is not charged, continue from group A
			}
			else
				chargeRestartCounter = 0; //Restart timeout reached (18 hrs)
				
			back_buffer[20] = bmsOn ? 0x03 : 0x20; //Show "plug" sign if chargers are on
			back_buffer[21] = 0x20;
			if (bmsOn)
				back_buffer[22] = bmsSelB ? 0xE2 : 0xE0; //Show "alpha" or "  beta" sign
			else
				back_buffer[22] = 0x20;
			back_buffer[18] = IS_CHARGE_END ? 0x02 : 0x01; //Show "empty" or "full" battery sign
		}
		else //On battery
		{
			bmsOn = 0;
			bmsSelB = 0;		
			
			temp32 = current; //x
			temp32 *= temp32; //x^2
			//Calculate voltage cut-off
			temp16 = temp32 / K_VCUT_a; //ax^2
			temp16 -= current / K_VCUT_b; //ax^2 - bx
			temp16 += K_VCUT_c; //V_cut = ax^2 - bx + c
			vSlaFail = (temp16 + 10) / 20;
			uint16_t voltage = accuVS[19]; //V_accu
			if (voltage <= temp16)
			{
				if (!countdown)
				{
					eeprom_write_byte(&slaFailIdx, slaIdxRam);
					DC_IGBT_off;
					_delay_us(66);
					DC_MOS_off;
				}
				else
					countdown--;
				temp16 = 0;
			}
			else
			{
				countdown = 5;
				voltage -= temp16; //V_accu - V_cut
				//Calculate voltage delta
				temp16 = current / K_VDELTA_b; //bx
				temp16 -= temp32 / K_VDELTA_a; //-ax^2 + bx
				//Voltage correction
				temp32 = voltage; //V_delta_curr
				temp32 *= VDELTA_REF; //V_delta_curr * V_delta_ref
				temp32 /= temp16 + K_VDELTA_c; //V_delta_corr
				temp32 += VCUT_REF;
				voltage = temp32; //Corrected battery voltage
				//Charge level approximation
				temp32 *= temp32; //x^2
				temp16 = temp32 / K_VSOC_a; //ax^2
				temp16 += K_VSOC_c; //ax^2 + c
				temp32 = voltage;
				temp32 *= K_VSOC_b; //bx<<16
				temp16 -= temp32 >> 16; //Charge level = ax^2 - bx + c
				if (temp16 > 99)
					temp16 = 99;	
			}
			back_buffer[18] = 0x02; //"Empty" battery sign
			uint2buffer(back_buffer + 20, 2, 2, temp16); //Charge in %
			back_buffer[22] = 0x25; //Percent sign
		}
	#endif
		if (!(dispIdx & 3))
		{
			//Calculate fan RPM
			if (captCS > 1)
			{
				captSS /= 10;
				temp32 = F_CPU * 3 / 8UL;
				temp32 *= captCS - 1;
				temp16 = temp32 / captSS;
			}
			else
				temp16 = 0;
			uint2buffer(back_buffer + 24, 4, 4, temp16); //Fan RPM			
		}

		//Send buffer to the display
		lcd_send_byte(0x80, 0); //First row
		lcd_send_buffer(back_buffer, 16);
		lcd_send_byte(0xC0, 0); //Second row
		lcd_send_buffer(back_buffer + 16, 16);
		//Calculate fan speed based on temperature delta
		tDelta >>= 6; //Range 0�120, if delta T = 30�C
		if (tDelta > T_DELTA)
			tDelta = T_DELTA;
	#ifndef TEST_VI
		if (bmsOn && !IS_CHARGE_END && tDelta < 10)
			tDelta = 10;
	#endif
		temp16 = (255 - FAN_min) * tDelta / T_DELTA + FAN_min;
		if ((fanLvl >= FAN_min && temp16 > fanLvl) || (fanLvl < FAN_min && temp16 > FAN_min))
			fanLvl = temp16; //Increase level
		else {
			tDelta += T_DELTA >> 4; //Lower bound for stepping down
			tDelta = (255 - FAN_min) * tDelta / T_DELTA + FAN_min;
			if (tDelta < fanLvl) //Both lower bounds are below
				fanLvl = temp16 > FAN_min ? temp16 : 0; //Decrease level
		}
    }
}
