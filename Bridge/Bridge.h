/* Bridge.h
 * Created: 26.11.2017 22:13:53
 *  Author: Андрей	*/

#ifndef BRIDGE_H_
#define BRIDGE_H_

#define F_CPU		3686400L
#include <util/delay.h>

#define U0RXen()	(PORTA &= ~(1 << PORTA3))
#define U0TXen()	(PORTA |= (1 << PORTA3))
#endif /* BRIDGE_H_ */