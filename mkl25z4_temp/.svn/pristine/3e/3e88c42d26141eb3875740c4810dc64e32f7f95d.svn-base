/*
 * Name: uart_irq_send.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "adc_bm.h"
#include "wait_bm.h"
#include "print.h"
#include "str.h"

#define UART1_EXPT_PRI		2u

#define TX_MSG_LEN_LIM		32u

char tx_msg[TX_MSG_LEN_LIM];
volatile uint8_t tx_msg_len, tx_msg_ind;

void __attribute__ ((interrupt)) UART1_IRQHandler(void)
{
	UART1->S1;

	if (tx_msg_ind < tx_msg_len) {
		UART1->D = tx_msg[tx_msg_ind++];
	} else {
		UART1->C2 = UART_C2_TE_MASK;
	}
}

int main(void)
{
	uint8_t pot_val;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	PORT_UART1_RX->PCR[IOIND_UART1_RX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_UART1_TX->PCR[IOIND_UART1_TX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	NVIC_SetPriority(UART1_IRQn, UART1_EXPT_PRI);
	NVIC_EnableIRQ(UART1_IRQn);

	UART1->BDH = UART_BDH_SBR(UART_SBR_115200BD_CLK24M >> 8u);
	UART1->BDL = UART_BDL_SBR(UART_SBR_115200BD_CLK24M);
	UART1->C2 = UART_C2_TE_MASK;

	while (1) {
		if (UART1->C2 & UART_C2_TIE_MASK) {
			continue;
		}

		pot_val = adc_bm_read(ADC_BM_MODE_8BIT_SE, ADC_CHAN_POT1);

		tx_msg_len = 0u;
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], "pot1: ");
		tx_msg_len += print_udec(&tx_msg[tx_msg_len], pot_val);
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], "  \r");

		tx_msg_ind = 0u;
		UART1->C2 = UART_C2_TIE_MASK | UART_C2_TE_MASK;

		wait_bm(WAIT_BM_100MSEC_FSYS48M, 1u);
	}

	return 0;
}
