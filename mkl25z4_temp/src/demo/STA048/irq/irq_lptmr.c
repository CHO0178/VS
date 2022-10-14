/*
 * Name: irq_lptmr.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "led.h"

#define DISP_PERIOD		15625ul

#define LPTMR_EXPT_PRI	2u

uint8_t led_val;

void __attribute__ ((interrupt)) LPTMR0_IRQHandler(void)
{
	LPTMR0->CSR = LPTMR_CSR_TCF_MASK | LPTMR_CSR_TIE_MASK | LPTMR_CSR_TEN_MASK;
	led_bindisp(++led_val);
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();

	LPTMR0->CSR = 0ul;
	NVIC_ClearPendingIRQ(LPTMR0_IRQn);

	NVIC_SetPriority(LPTMR0_IRQn, LPTMR_EXPT_PRI);
	NVIC_EnableIRQ(LPTMR0_IRQn);

	LPTMR0->PSR = LPTMR_PSR_PRESCALE(LPTMR_PSR_PRS_VAL_D256_F128) |
					LPTMR_PSR_PCS(LPTMR_PSR_PCS_VAL_OSCERCLK);
	LPTMR0->CMR = DISP_PERIOD - 1u;
	LPTMR0->CSR = LPTMR_CSR_TIE_MASK | LPTMR_CSR_TEN_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
