/*
 * Name: gpio_pdir.c
 * Author: Martin Stankus
 *
 * Demo utilises BNC J9.
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"

//do a ctrl-click on symbolic constants to see their meaning

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	//the IOMASK_J9 bit of the GPIO_J9->PDDR register is already 0 (input)
	PORT_J9->PCR[IOIND_J9] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	GPIO_LEDG->PDDR |= IOMASK_LEDG;
	PORT_LEDG->PCR[IOIND_LEDG] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	while (1) {
		if (GPIO_J9->PDIR & IOMASK_J9) {
			GPIO_LEDG->PSOR = IOMASK_LEDG;
		} else {
			GPIO_LEDG->PCOR = IOMASK_LEDG;
		}

		wdog_refresh();
	}

	return 0;
}
