/*
 * Name: uart_irq_recv_send.c
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

#define UART_C2_COMPVAL		(UART_C2_RIE_MASK | UART_C2_TE_MASK | UART_C2_RE_MASK)

#define TX_MSG_LEN_LIM		64u

char tx_msg[TX_MSG_LEN_LIM];
volatile uint8_t tx_msg_len, tx_msg_ind;

void __attribute__ ((interrupt)) UART1_IRQHandler(void)
{
	//receive direction
	if (UART1->S1 & UART_S1_RDRF_MASK) {
		switch (UART1->D) {
		case 'r':
			GPIO_LEDR->PTOR	= IOMASK_LEDR;
			break;
		case 'g':
			GPIO_LEDG->PTOR	= IOMASK_LEDG;
			break;
		case 'b':
			GPIO_LEDB->PTOR	= IOMASK_LEDB;
			break;
		default:
			break;
		}
	}

	//send direction
	if ((UART1->C2 & UART_C2_TIE_MASK) && (UART1->S1 & UART_S1_TDRE_MASK)) {
		if (tx_msg_ind < tx_msg_len) {
			UART1->D = tx_msg[tx_msg_ind++];
		} else {
			UART1->C2 = UART_C2_COMPVAL;
		}
	}
}

int main(void)
{
	uint8_t pot1_val, pot2_val;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	//led init
	GPIO_LEDR->PSOR	= IOMASK_LEDR;
	GPIO_LEDR->PDDR |= IOMASK_LEDR;
	PORT_LEDR->PCR[IOIND_LEDR] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	GPIO_LEDG->PSOR	= IOMASK_LEDG;
	GPIO_LEDG->PDDR |= IOMASK_LEDG;
	PORT_LEDG->PCR[IOIND_LEDG] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	GPIO_LEDB->PSOR	= IOMASK_LEDB;
	GPIO_LEDB->PDDR |= IOMASK_LEDB;
	PORT_LEDB->PCR[IOIND_LEDB] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	//uart init
	PORT_UART1_RX->PCR[IOIND_UART1_RX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_UART1_TX->PCR[IOIND_UART1_TX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	NVIC_SetPriority(UART1_IRQn, UART1_EXPT_PRI);
	NVIC_EnableIRQ(UART1_IRQn);

	UART1->BDH = UART_BDH_SBR(UART_SBR_115200BD_CLK24M >> 8u);
	UART1->BDL = UART_BDL_SBR(UART_SBR_115200BD_CLK24M);
	UART1->C2 = UART_C2_COMPVAL;

	while (1) {
		if (UART1->C2 & UART_C2_TIE_MASK) {
			continue;
		}

		pot1_val = adc_bm_read(ADC_BM_MODE_8BIT_SE, ADC_CHAN_POT1);
		pot2_val = adc_bm_read(ADC_BM_MODE_8BIT_SE, ADC_CHAN_POT2);

		tx_msg_len = 0u;
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], "pot1: ");
		tx_msg_len += print_udec(&tx_msg[tx_msg_len], pot1_val);
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], ", pot2: ");
		tx_msg_len += print_udec(&tx_msg[tx_msg_len], pot2_val);
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], "    \r");

		tx_msg_ind = 0u;
		UART1->C2 = UART_C2_TIE_MASK | UART_C2_COMPVAL;

		wait_bm(WAIT_BM_100MSEC_FSYS48M, 1u);
	}

	return 0;
}
