/*
 * Name: adc_poll.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "led.h"

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();

	ADC0->CFG1 = ADC_CFG1_ADIV(ADC_CFG1_ADIV_VAL_DIV4);
	ADC0->SC3 = ADC_SC3_ADCO_MASK;
	ADC0->SC1[0u] = ADC_CHAN_POT1;

	//bandgap voltage meas
	//ADC0->SC1[0u] = ADC_CHAN_BGAP;
	//PMC->REGSC = PMC_REGSC_BGBE_MASK;

	while (1) {
		if (ADC0->SC1[0u] & ADC_SC1_COCO_MASK) {
			led_bgraph(ADC0->R[0u] * 0.035f);
			//led_bindisp(ADC0->R[0u]);
		}
		wdog_refresh();
	}

	return 0;
}
