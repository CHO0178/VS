/*
 * Name: irq_btn.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"

#define PORTA_EXPT_PRI	2u

void __attribute__ ((interrupt)) PORTA_IRQHandler(void)
{
	static uint8_t led_val = 0u;

	PORT_BTN1->ISFR = IOMASK_BTN1;

	if (led_val) {
		led_val = 0u;
		GPIO_LEDR->PSOR = IOMASK_LEDR;
	} else {
		led_val = 1u;
		GPIO_LEDR->PCOR = IOMASK_LEDR;
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	GPIO_LEDR->PSOR = IOMASK_LEDR;
	GPIO_LEDR->PDDR |= IOMASK_LEDR;
	PORT_LEDR->PCR[IOIND_LEDR] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	NVIC_SetPriority(PORTA_IRQn, PORTA_EXPT_PRI);
	NVIC_EnableIRQ(PORTA_IRQn);

	PORT_BTN1->PCR[IOIND_BTN1] = PORT_PCR_IRQC(PORT_PCR_IRQC_VAL_INT_EFALL) |
			PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	while (1) {
		wdog_refresh();
	}

	return 0;
}
