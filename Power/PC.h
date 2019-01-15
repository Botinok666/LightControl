/* PC.h
 * Created: 25.01.2018 17:01:39
 *  Author: Andrew */

#ifndef PC_H_
#define PC_H_
#define F_CPU	8000000L

#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

#define Bat_100p	31802
#define Bat_12p		25998
#define Bat_1p		23482
#define Bat_ESR		43671	//32 pieces * 23mOhm
#define Charge_HDiv	66		//100-12% divider
#define Charge_LDiv	210		//1-12% divider
#define TSEN_MIN	(50 << 2)		//50°C
#define FanMin		24		//15% DC
#define TSEN_MAX	(80 << 2)		//80°C
#define FanMax		160		//100% DC

#define THS1_TWI_RD		0x91
#define THS1_TWI_WR		0x90
#define THS2_TWI_RD		0x9F
#define THS2_TWI_WR		0x9E
#define IOUT_TWI_RD		0x81
#define IOUT_TWI_WR		0x80

#define We_Are_OffLine	(PORTD & (1 << PORTD1))
#define DC_MOS_On		(PORTD |= (1 << PORTD1))
#define DC_MOS_Off		(PORTD &= ~(1 << PORTD1))
#define AC_MOS_On		(PORTD |= (1 << PORTD0))
#define AC_MOS_Off		(PORTD &= ~(1 << PORTD0))

#define BMS_Sel_A		(PORTB &= ~(1 << PORTB2))
#define BMS_Sel_B		(PORTB |= (1 << PORTB2))
#define BMS_B_Selected	(PORTB & (1 << PORTB2))
#define Charger_On		(PORTB |= (1 << PORTB1))
#define Charger_Off		(PORTB &= ~(1 << PORTB1))

#endif /* PC_H_ */