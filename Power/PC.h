/* PC.h
 * Created: 25.01.2018 17:01:39
 *  Author: Andrew */

#ifndef PC_H_
#define PC_H_

#define F_CPU	8000000L
#include <util/delay.h>
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

#define DC_MIN		1926
#define TSEN_ZERO	621 //0°C
#define TSEN_MIN	1241 //50°C
#define TSEN_MAX	1614 //80°C

#define PWM_freq	25000
#define PWM_max		(F_CPU / (2L * PWM_freq))
#define FanMin		((25 * PWM_max) / 100)
#define FanMax		((95 * PWM_max) / 100)

#define DC_to_Load		(PORTA |= (1 << PORTA7))
#define AC_to_Load		(PORTA &= (1 << PORTA7))
#define We_Are_OffLine	(PORTA & (1 << PORTA7))
#define DC_MOS_On		(PORTA |= (1 << PORTA4))
#define DC_MOS_Off		(PORTA &= ~(1 << PORTA4))
#define ICL_Active		(PORTB &= ~(1 << PORTB2))
#define ICL_Shunted		(PORTB |= (1 << PORTB2))
#define BMS_Sel_A		(PORTA &= ~(1 << PORTA1))
#define BMS_Sel_B		(PORTA |= (1 << PORTA1))
#define BMS_B_Selected	(PORTA & (1 << PORTA1))
#define Charger_On		(PORTB |= (1 << PORTB0))
#define Charger_Off		(PORTB &= ~(1 << PORTB0))

#endif /* PC_H_ */