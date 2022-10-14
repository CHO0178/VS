/*
 * Name: expt_race_cond.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"
#include "wdog.h"
#include "io_dock.h"
#include "adc_defs.h"
#include "led.h"

#define ADC_EXPT_PRI		2
#define PIT_EXPT_PRI		2

#define PIT0_MOD			240000ul

#define VAR_GLOB_PROTECT	1

volatile uint8_t var_glob1, var_glob2;

void __attribute__ ((interrupt)) ADC0_IRQHandler(void)
{
	led_bgraph(ADC0->R[0] * 0.035f);
}

void __attribute__ ((interrupt)) PIT_IRQHandler(void)
{
	PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
	ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_CHAN_POT1;

	if (var_glob1 != var_glob2) {
		__BKPT(0);
	}
}

void resource_access(void)
{
#if VAR_GLOB_PROTECT
	uint32_t primask;
	primask = __get_PRIMASK();
	__disable_irq();
#endif

	var_glob1 = ~var_glob1;
	var_glob2 = ~var_glob2;

#if VAR_GLOB_PROTECT
	__set_PRIMASK(primask);
#endif
}

int main(void)
{
	wdog_set(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();

	NVIC_SetPriority(ADC0_IRQn, ADC_EXPT_PRI);
	NVIC_EnableIRQ(ADC0_IRQn);

	NVIC_SetPriority(PIT_IRQn, PIT_EXPT_PRI);
	NVIC_EnableIRQ(PIT_IRQn);

	ADC0->CFG1 = ADC_CFG1_ADIV(ADC_CFG1_ADIV_VAL_DIV4);

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0].LDVAL = PIT0_MOD - 1;
	PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

	while (1) {
		resource_access();
		wdog_refresh();
	}

	return 0;
}
