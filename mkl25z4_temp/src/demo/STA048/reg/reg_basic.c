/*
 * Name: reg_basic.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "wdog.h"

int main(void)
{
	wdog_init(WDOG_CONF_DIS);

	GPIOB->PDDR |= (1ul << 18u);
	PORTB->PCR[18u] = PORT_PCR_MUX(1u);

	while (1) {
		GPIOB->PTOR = (1ul << 18u);
		for (int wait = 0; wait < 500000; wait++);
	}

	return 0;
}
