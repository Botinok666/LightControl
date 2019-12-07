/* twi.c
 * Created: 07.12.2018 21:25:48
 *  Author: Andrew */
#include <avr/io.h>
#include "twi.h"

#define TWI_SAFE_WAIT

void inline twi_wait(void) {
	uint16_t temp = TCNT1; //1MHz -> 1µs tick
	temp += 4000; //4ms timeout
	if (temp < TCNT1) //overflow
	{
		while ((TCNT1 > temp) & !(TWCR & (1 << TWINT))); //Wait overflow of TCNT1
		while ((TCNT1 < temp) & !(TWCR & (1 << TWINT))); 
	}
	else
		while ((TCNT1 < temp) & !(TWCR & (1 << TWINT)));
}

void twi_start(void) {
	TWCR = (1 << TWEN) | (1 << TWSTA) | (1 << TWINT);
#ifdef TWI_SAFE_WAIT
	twi_wait();
#else
	while (!(TWCR & (1 << TWINT)));
#endif
}

void twi_send(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWEN) | (1 << TWINT);
#ifdef TWI_SAFE_WAIT
	twi_wait();
#else
	while (!(TWCR & (1 << TWINT)));
#endif
}

void inline twi_stop(void) {
	TWCR = (1 << TWEN) | (1 << TWSTO) | (1 << TWINT);
	while (!(TWCR & 0x10));
}

uint8_t twi_receive(uint8_t ack) {
	if (ack > 0)
		TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT);
	else
		TWCR = (1 << TWEN) | (1 << TWINT);
#ifdef TWI_SAFE_WAIT
	twi_wait();
#else
	while (!(TWCR & (1 << TWINT)));
#endif
	return TWDR;
}

void twi_set_register(uint8_t device, uint8_t pointer) {
	twi_start();
	twi_send(device);
	twi_send(pointer);
}