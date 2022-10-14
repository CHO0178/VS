/*
 * Name: delay_test.c
 * Author: Martin Stankus
 *
 */

#include "wdog.h"
#include "delay.h"

#include "soc_def.h"
#include "res_alloc.h"

int main(void)
{
	wdog_init(WDOG_CONF_DIS);

	FGPIO_J8->PCOR = IOMASK_J8;
	FGPIO_J8->PDDR |= IOMASK_J8;
	PORT_J8->PCR[IOIND_J8] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	FGPIO_J8->PSOR = IOMASK_J8;
	delay(1000000ul);
	FGPIO_J8->PCOR = IOMASK_J8;

	while (1) {
	}

	return 0;
}
