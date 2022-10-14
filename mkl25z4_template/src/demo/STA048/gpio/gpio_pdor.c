/*
 * Name: gpio_pdor.c
 * Author: Martin Stankus
 *
 * Demo utilises BNC J8.
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"

//do a ctrl-click on symbolic constants to see their meaning

int main(void)
{
	uint8_t state = 0u;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	GPIO_J8->PDDR |= IOMASK_J8;
	PORT_J8->PCR[IOIND_J8] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	while (1) {
		if (state) {
			state = 0u;
			GPIO_J8->PDOR &= ~IOMASK_J8;
		} else {
			state = 1u;
			GPIO_J8->PDOR |= IOMASK_J8;
		}

		wdog_refresh();
	}

	return 0;
}
