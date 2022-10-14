/*
 * Name: uart_irq_recv.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"

#define UART1_EXPT_PRI		2u

void __attribute__ ((interrupt)) UART1_IRQHandler(void)
{
	UART1->S1;

	if (UART1->D == 't') {
		GPIO_LEDR->PTOR	= IOMASK_LEDR;
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	//led init
	GPIO_LEDR->PSOR	= IOMASK_LEDR;
	GPIO_LEDR->PDDR |= IOMASK_LEDR;
	PORT_LEDR->PCR[IOIND_LEDR] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	//uart init
	PORT_UART1_RX->PCR[IOIND_UART1_RX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_UART1_TX->PCR[IOIND_UART1_TX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	NVIC_SetPriority(UART1_IRQn, UART1_EXPT_PRI);
	NVIC_EnableIRQ(UART1_IRQn);

	UART1->BDH = UART_BDH_SBR(UART_SBR_115200BD_CLK24M >> 8u);
	UART1->BDL = UART_BDL_SBR(UART_SBR_115200BD_CLK24M);
	UART1->C2 = UART_C2_RIE_MASK | UART_C2_RE_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
