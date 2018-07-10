/* PC.h
 * Created: 25.01.2018 17:01:39
 *  Author: Andrew */

#ifndef PC_H_
#define PC_H_

#define F_CPU	4000000L
#include <util/delay.h>
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

#define DC_MIN		1926
#define TSEN_ZERO	621
#define TSEN_MIN	1365
#define TSEN_MAX	1614

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

#define Set_Tick_flag	(GPIOR0 |= (1 << GPIOR00))
#define Clear_Tick_flag	(GPIOR0 &= ~(1 << GPIOR00))
#define Is_Tick_set		(GPIOR0 & (1 << GPIOR00))
#define Set_Discharged_flag		(GPIOR0 |= (1 << GPIOR01))
#define Clear_Discharged_flag	(GPIOR0 &= ~(1 << GPIOR01))
#define Is_Battery_discharged	(GPIOR0 & (1 << GPIOR01))
#define Set_ChargeReq_flag		(GPIOR0 |= (1 << GPIOR02))
#define Clear_ChargeReq_flag	(GPIOR0 &= ~(1 << GPIOR02))
#define Is_Charge_Required		(GPIOR0 & (1 << GPIOR02))
#endif /* PC_H_ */