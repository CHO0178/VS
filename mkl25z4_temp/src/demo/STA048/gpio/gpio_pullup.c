/*
 * Name: gpio_pullup.c
 * Author: Martin Stankus
 *
 * Demo utilises BNC J8.
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "wait_bm.h"

//do a ctrl-click on symbolic constants to see their meaning

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	//the IOMASK_J8 bit of the GPIO_J8->PDOR register is already 0 (low output)
	PORT_J8->PCR[IOIND_J8] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO) |
			PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

	while (1) {
		//PDDR = 1 (output) -> low output
		GPIO_J8->PDDR |= IOMASK_J8;
		wait_bm(WAIT_BM_10MSEC_FSYS48M, 1u);

		//PDDR = 0 (input) -> high-z (high output because of the pull-up resistor)
		GPIO_J8->PDDR &= ~IOMASK_J8;
		wait_bm(WAIT_BM_10MSEC_FSYS48M, 1u);
	}

	return 0;
}
