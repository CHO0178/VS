/*
 * Name: uart_irq_sync.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"

#define PIT_EXPT_PRI			2u
#define UART1_EXPT_PRI			2u

#define UART_SBR_200BD_CLK24M	7500u
#define UART_SBR_600BD_CLK24M	2500u

#define IDLE_DELAY_200BD		4000000ul
#define IDLE_DELAY_600BD		1500000ul

#define IDLE_DELAY				IDLE_DELAY_200BD
#define UART_SBR				UART_SBR_200BD_CLK24M

void __attribute__ ((interrupt)) UART1_IRQHandler(void)
{
#if 1
	static uint8_t tx_cnt;

	tx_cnt++;

	if (tx_cnt == 3u) {
		tx_cnt = 0u;

		UART1->C2 = UART_C2_TE_MASK;

		PIT->MCR = PIT_MCR_FRZ_MASK;
		PIT->CHANNEL[0u].LDVAL = IDLE_DELAY - 1u;
		PIT->CHANNEL[0u].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

		return;
	}
#endif

	UART1->S1;
	UART1->D = 0x5A;
}

void __attribute__ ((interrupt)) PIT_IRQHandler(void)
{
	PIT->CHANNEL[0u].TFLG = PIT_TFLG_TIF_MASK;
	PIT->MCR = 0ul;

	UART1->C2 = UART_C2_TIE_MASK | UART_C2_TE_MASK;
}

int main(void)
{
	wdog_init(WDOG_CONF_DIS);

	NVIC_SetPriority(PIT_IRQn, PIT_EXPT_PRI);
	NVIC_EnableIRQ(PIT_IRQn);

	PORT_UART1_RX->PCR[IOIND_UART1_RX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_UART1_TX->PCR[IOIND_UART1_TX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	NVIC_SetPriority(UART1_IRQn, UART1_EXPT_PRI);
	NVIC_EnableIRQ(UART1_IRQn);

	UART1->BDH = UART_BDH_SBR(UART_SBR >> 8u);
	UART1->BDL = UART_BDL_SBR(UART_SBR);
	UART1->C2 = UART_C2_TIE_MASK | UART_C2_TE_MASK;

	while (1) {
	}

	return 0;
}
