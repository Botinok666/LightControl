/*
 * twi.h
 *
 * Created: 07.12.2018 21:27:44
 *  Author: Andrew
 */
#include "stdint-gcc.h"
#ifndef TWI_H_
#define TWI_H_

void twi_start(void);
void twi_send(uint8_t);
void twi_stop(void);
uint8_t twi_receive(uint8_t);

#endif /* TWI_H_ */