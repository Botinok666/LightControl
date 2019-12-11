/* main.h
 * Created: 7/4/2019 16:08:06
 *  Author: Andrew */ 

#ifndef _MAIN_H_
#define _MAIN_H_

#define AD7416_wr		0x90
#define AD7416_rd		0x91
#define INA219_wr		0x8A
#define INA219_rd		0x8B
#define MCP3421_wr		0xD0
#define MCP3421_rd		0xD1
#define PCF8574_wr		0x4C
#define PCF8574_rd		0x4D

#define T_LOWER			(50 << 8)
#define T_UPPER			(80 << 8)
#define T_DELTA			((T_UPPER - T_LOWER) >> 6)
#define FAN_min			(255 * 25 / 100)

#define VSEN_div		3049
#define V_SLA_FAIL		90	//9V for single battery

#define WE_ARE_ONLINE	(PORTC & (1 << PORTC3))
#define WE_ARE_OFFLINE	(!WE_ARE_ONLINE)
#define ONLY_ICL_IS_ON	((PORTC & ((1 << PORTC2) | (1 << PORTC1))) == (1 << PORTC1))
#define SHDN_THRESHOLD	(105 * 20) //10.5V for single battery

#define IS_CHARGE_END	((PIND & (1 << PIND7)) == 0)
#define CHARGE_RESTART	64800	//Approx. 18 hours
#define CHARGE_CYCLE	3600	//Approx. 1 hour
#define CHARGE_DELAY	5		//Approx. 5 seconds

#define DC_IGBT_on		(PORTC |= (1 << PORTC2))
#define DC_IGBT_off		(PORTC &= ~(1 << PORTC2))
#define DC_MOS_on		(PORTC |= (1 << PORTC1))
#define DC_MOS_off		(PORTC &= ~(1 << PORTC1))
#define AC_IGBT_on		(PORTC |= (1 << PORTC3))
#define AC_IGBT_off		(PORTC &= ~(1 << PORTC3))

#define ALERT_on		(PORTB |= (1 << PORTB1))
#define ALERT_off		(PORTB &= ~(1 << PORTB1))

#endif /* MAIN_H_ */