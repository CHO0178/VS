/*
 * Name: periph_survey.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "expt.h"
#include "print.h"
#include "str.h"
#include "term.h"

#include "probe.h"

static void msg_send(char *msg, unsigned int msg_len)
{
	DMA0->DMA[0u].SAR = (uint32_t) msg;
	DMA0->DMA[0u].DSR_BCR = DMA_DSR_BCR_BCR(msg_len);
	DMA0->DMA[0u].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_SINC_MASK | DMA_DCR_SSIZE(1u) | DMA_DCR_DSIZE(1u) | DMA_DCR_D_REQ_MASK;

	while (!(DMA0->DMA[0u].DSR_BCR & DMA_DSR_BCR_DONE_MASK));
	DMA0->DMA[0u].DSR_BCR = DMA_DSR_BCR_DONE_MASK;
}

int main(void)
{
	unsigned int msg_len, slot_ind, probe_ind, hit_cnt;
	char msg[64];
	uint8_t dummy;

	wdog_init(WDOG_CONF_DIS);

	PORT_UART1_RX->PCR[IOIND_UART1_RX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_UART1_TX->PCR[IOIND_UART1_TX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	UART1->BDH = UART_BDH_SBR(UART_SBR_115200BD_CLK24M >> 8u);
	UART1->BDL = UART_BDL_SBR(UART_SBR_115200BD_CLK24M);
	UART1->C4 = UART_C4_TDMAS_MASK;
	UART1->C2 = UART_C2_TIE_MASK | UART_C2_TE_MASK;

	DMAMUX0->CHCFG[0u] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_UART1_TX);
	DMA0->DMA[0u].DAR = (uint32_t) &UART1->D;

	expt_vect_set(HardFault_IRQn, probe_hardfault_handler);

	for (slot_ind = 0u; slot_ind < 256u; slot_ind++) {

		msg_len = 0u;
		msg_len += strcpy(&msg[msg_len], "Slot index: ");
		msg_len += print_udec(&msg[msg_len], slot_ind);
		msg_len += strcpy(&msg[msg_len], "\n");

		hit_cnt = 0u;

		for (probe_ind = 0u; probe_ind < 4096u; probe_ind++) {
			if (probe(&dummy, (uint8_t *) (0x40000000ul | (slot_ind << 12u) | probe_ind))) {
				hit_cnt++;
			}
		}

		if (hit_cnt > 0u) {
			msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_FGC_RED));
			msg_len += strcpy(&msg[msg_len], "Probe hits: ");
			msg_len += print_udec(&msg[msg_len], hit_cnt);
			msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_FGC_WHITE));
			msg_len += strcpy(&msg[msg_len], "\n");
		}

		msg_len += strcpy(&msg[msg_len], "\n");

		msg_send(msg, msg_len);
	}

	while (1) {
		__BKPT(0);
	}

	return 0;
}
