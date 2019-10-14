/* LC.h
 * Created: 23.01.2018 22:01:22
 *  Author: Andrew */

#ifndef LC_H_
#define LC_H_

#define BOARD_A
#define F_CPU	32000000L
#include <util/delay.h>
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))

#define LINK_VALID_MIN	5001
#define LINK_VALID_MAX	31577
#define LINK_ON_BARRIER	6981
#define LINK_1PERC_CODE	8592
#define LINK_SCALE		84

#define MSEN_VALID_MIN	20
#define MSEN_SEN1_TRIG	75
#define MSEN_SEN2_TRIG	100

#ifdef BOARD_A
#define CmdLC		0x21
#define GetStatus	0x21
#define GetConfig	0x22
#define SetConfig	0x23
#define GetOnTime	0x24
#define CmdUC		0x24
#else //BOARD_B
#define CmdLC		0x31
#define GetStatus	0x31
#define GetConfig	0x32
#define SetConfig	0x33
#define GetOnTime	0x34
#define CmdUC		0x34
#endif //BOARD_A

#define UCRXen()	(PORTC.OUTCLR = PIN1_bm)

#endif /* LC_H_ */