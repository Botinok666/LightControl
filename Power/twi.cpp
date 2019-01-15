/*
 * twi.c
 *
 * Created: 07.12.2018 21:25:48
 *  Author: Andrew
 */
#include <avr/io.h>

void twi_start(void) {
	TWCR = (1 << TWEN) | (1 << TWSTA) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

void twi_send(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

void twi_stop(void) {
	TWCR = (1 << TWEN) | (1 << TWSTO) | (1 << TWINT);
	while (!(TWCR & 0x10));
}

uint8_t twi_receive(uint8_t ack) {
	if (ack > 0)
		TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT);
	else
		TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}