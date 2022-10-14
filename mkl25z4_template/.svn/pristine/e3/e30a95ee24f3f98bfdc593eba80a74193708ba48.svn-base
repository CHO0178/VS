/*
 * Name: uart_send.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "wait_bm.h"

int main(void)
{
	char tx_data = 'A';

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	PORT_UART1_RX->PCR[IOIND_UART1_RX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_UART1_TX->PCR[IOIND_UART1_TX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	UART1->BDH = UART_BDH_SBR(UART_SBR_115200BD_CLK24M >> 8u);
	UART1->BDL = UART_BDL_SBR(UART_SBR_115200BD_CLK24M);
	UART1->C2 = UART_C2_TE_MASK;

	while (1) {
		if (UART1->S1 & UART_S1_TDRE_MASK) {
			UART1->D = tx_data;

			tx_data++;
			if (tx_data > 'Z') {
				tx_data = 'A';
			}

			wait_bm(WAIT_BM_500MSEC_FSYS48M, 1u);
		}
	}

	return 0;
}
