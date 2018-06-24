/* AC.h
 * Created: 24.11.2017 20:00:02
 *  Author: Andrew	*/

#ifndef AC_H_
#define AC_H_

#define F_CPU		7372800L
#include <util/delay.h>

#define PWM_freq	25000
#define PWM_max		(F_CPU / PWM_freq - 1)
#define FanMin		((25 * PWM_max) >> 8)
#define FanMax		((250 * PWM_max) >> 8)

#define CmdLC		0x11
#define GetStatus	0x11
#define GetConfig	0x12
#define SetConfig	0x13
#define CmdUC		0x13

#define U0RXen()	PORTA &= ~(1 << PORTA5)
#define SelChA()	PORTA |= (1 << PORTA6)
#define SelChB()	PORTA &= ~(1 << PORTA6)

#define Idiv_x1mA	33

#endif /* AC_H_ */