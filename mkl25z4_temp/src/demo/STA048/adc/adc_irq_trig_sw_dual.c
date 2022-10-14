/*
 * Name: adc_irq_trig_sw_dual.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "led.h"

#define ADC_EXPT_PRI		2u
#define PIT_EXPT_PRI		2u

#define PIT0_MOD			240000ul

uint8_t phase;

void __attribute__ ((interrupt)) ADC0_IRQHandler(void)
{
	if (!phase) {
		phase = 1u;
		led_low_bgraph(ADC0->R[0u] * 0.0175f);
		ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_CHAN_POT2;
	} else {
		phase = 0u;
		led_high_bgraph(ADC0->R[0u] * 0.0175f);
	}
}

void __attribute__ ((interrupt)) PIT_IRQHandler(void)
{
	PIT->CHANNEL[0u].TFLG = PIT_TFLG_TIF_MASK;
	ADC0->SC1[0u] = ADC_SC1_AIEN_MASK | ADC_CHAN_POT1;
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();

	NVIC_SetPriority(ADC0_IRQn, ADC_EXPT_PRI);
	NVIC_EnableIRQ(ADC0_IRQn);

	NVIC_SetPriority(PIT_IRQn, PIT_EXPT_PRI);
	NVIC_EnableIRQ(PIT_IRQn);

	ADC0->CFG1 = ADC_CFG1_ADIV(ADC_CFG1_ADIV_VAL_DIV4);

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0u].LDVAL = PIT0_MOD - 1u;
	PIT->CHANNEL[0u].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
