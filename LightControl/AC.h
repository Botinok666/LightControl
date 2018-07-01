/* AC.h
 * Created: 24.11.2017 20:00:02
 *  Author: Andrew	*/

#ifndef AC_H_
#define AC_H_

#define F_CPU		7372800L
#include <util/delay.h>
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

#define PWM_freq	25000
#define PWM_max		(F_CPU / (2L * PWM_freq))
#define FanMin		((24 * PWM_max) / 100)
#define FanMax		((92 * PWM_max) / 100)

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