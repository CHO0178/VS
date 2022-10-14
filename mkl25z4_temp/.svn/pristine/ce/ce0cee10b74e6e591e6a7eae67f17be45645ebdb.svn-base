/*
 * Name: expt_vect_swap.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"
#include "wdog.h"
#include "io_dock.h"
#include "adc_defs.h"
#include "led.h"
#include "expt.h"
#include "wait.h"

#define ADC_EXPT_PRI		2
#define PIT_EXPT_PRI		2

#define PIT0_MOD			240000ul

#define WAIT_LIMIT			1000000ul

void __attribute__ ((interrupt)) adc0_irqhandler_bgraph(void)
{
	led_bgraph(ADC0->R[0] * 0.035f);
}

void __attribute__ ((interrupt)) adc0_irqhandler_bindisp(void)
{
	led_bindisp(ADC0->R[0]);
}

void __attribute__ ((interrupt)) PIT_IRQHandler(void)
{
	PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
	ADC0->SC1[0] = ADC_SC1_AIEN_MASK | ADC_CHAN_POT1;
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

	expt_vect_set(ADC0_IRQn, adc0_irqhandler_bgraph);

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0].LDVAL = PIT0_MOD - 1;
	PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

	while (1) {
		wait(WAIT_2000MSEC_FSYS48M);
		expt_vect_set(ADC0_IRQn, adc0_irqhandler_bindisp);
		wait(WAIT_2000MSEC_FSYS48M);
		expt_vect_set(ADC0_IRQn, adc0_irqhandler_bgraph);
	}

	return 0;
}
