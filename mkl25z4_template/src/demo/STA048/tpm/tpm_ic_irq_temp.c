/*
 * Name: tpm_ic_irq_temp.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "print.h"
#include "str.h"
#include "term.h"
#include "led.h"

#define TPM0_EXPT_PRI		1u
#define UART1_EXPT_PRI		2u

#define MSG_LEN_LIM			128u

char msg[MSG_LEN_LIM];
volatile uint8_t msg_len, msg_ind;

volatile uint32_t time_high, time_low;

void __attribute__ ((interrupt)) TPM0_IRQHandler(void)
{
	static uint32_t time, time_chan;
	static uint8_t edge_cnt;

	if (TPM0->STATUS & TPM_STATUS_TOF_MASK) {
		TPM0->STATUS = TPM_STATUS_TOF_MASK;
		time += 65536ul;
	} else {
		TPM0->STATUS = 1ul << TPMCH_TMP05;

		if (TPM0->CONTROLS[TPMCH_TMP05].CnSC & TPM_CnSC_ELSB_MASK) {
			time_high = time + TPM0->CONTROLS[TPMCH_TMP05].CnV - time_chan;
			TPM0->CONTROLS[TPMCH_TMP05].CnSC = 0ul;
			TPM0->CONTROLS[TPMCH_TMP05].CnSC = TPM_CnSC_CHIE_MASK | TPM_CnSC_ELSA_MASK;
		} else {
			time_low = time + TPM0->CONTROLS[TPMCH_TMP05].CnV - time_chan;
			TPM0->CONTROLS[TPMCH_TMP05].CnSC = 0ul;
			TPM0->CONTROLS[TPMCH_TMP05].CnSC = TPM_CnSC_CHIE_MASK | TPM_CnSC_ELSB_MASK;
		}

		time_chan = TPM0->CONTROLS[TPMCH_TMP05].CnV;
		time = 0ul;

		led_bindisp(edge_cnt++);
	}
}

void __attribute__ ((interrupt)) UART1_IRQHandler(void)
{
	UART1->S1;

	if (msg_ind < msg_len) {
		UART1->D = msg[msg_ind++];
	} else {
		UART1->C2 = UART_C2_TE_MASK;
	}
}

int main(void)
{
	int32_t temp;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();

	PORT_UART1_RX->PCR[IOIND_UART1_RX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_UART1_TX->PCR[IOIND_UART1_TX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	NVIC_SetPriority(UART1_IRQn, UART1_EXPT_PRI);
	NVIC_EnableIRQ(UART1_IRQn);

	UART1->BDH = UART_BDH_SBR(UART_SBR_115200BD_CLK24M >> 8u);
	UART1->BDL = UART_BDL_SBR(UART_SBR_115200BD_CLK24M);
	UART1->C2 = UART_C2_TE_MASK;

	NVIC_SetPriority(TPM0_IRQn, TPM0_EXPT_PRI);
	NVIC_EnableIRQ(TPM0_IRQn);

	PORT_TMP05->PCR[IOIND_TMP05] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	TPM0->CONTROLS[TPMCH_TMP05].CnSC = TPM_CnSC_CHIE_MASK | TPM_CnSC_ELSB_MASK;
	TPM0->SC = TPM_SC_TOIE_MASK | TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) |
				TPM_SC_PS(TPM_SC_PS_VAL_D16);

	while (1) {
		if (UART1->C2 & UART_C2_TIE_MASK) {
			continue;
		}

		msg_len = 0u;

		msg_len += strcpy(&msg[msg_len], TERM_CU_OFF);

		msg_len += strcpy(&msg[msg_len], TERM_CUP(1,1));
		msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_ULINE_ON));
		msg_len += strcpy(&msg[msg_len], "THERMOMETER");
		msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_ULINE_OFF));
		msg_len += strcpy(&msg[msg_len], TERM_EL(TERM_EL_ARG_CUR_TO_END));

		temp = 421 - (751 * (((float) time_high) / time_low));

		msg_len += strcpy(&msg[msg_len], TERM_CUP(2,1));
		msg_len += strcpy(&msg[msg_len], "temperature: ");
		msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_FGC_RED));
		msg_len += print_dec(&msg[msg_len], temp);
		msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_FGC_WHITE));
		msg_len += strcpy(&msg[msg_len], " [deg. C]");
		msg_len += strcpy(&msg[msg_len], TERM_EL(TERM_EL_ARG_CUR_TO_END));

		msg_ind = 0u;
		UART1->C2 = UART_C2_TIE_MASK | UART_C2_TE_MASK;

		wdog_refresh();
	}

	return 0;
}
