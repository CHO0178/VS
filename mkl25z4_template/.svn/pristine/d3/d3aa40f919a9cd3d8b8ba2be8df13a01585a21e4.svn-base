/*
 * Name: adc_irq_trig_hw_pit.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "led.h"

#define ADC_EXPT_PRI		2u

#define PIT0_MOD			240000ul

void __attribute__ ((interrupt)) ADC0_IRQHandler(void)
{
	led_bgraph(ADC0->R[0u] * 0.035f);
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();

	NVIC_SetPriority(ADC0_IRQn, ADC_EXPT_PRI);
	NVIC_EnableIRQ(ADC0_IRQn);

	ADC0->CFG1 = ADC_CFG1_ADIV(ADC_CFG1_ADIV_VAL_DIV4);
	ADC0->SC2 = ADC_SC2_ADTRG_MASK;
	ADC0->SC1[0u] = ADC_SC1_AIEN_MASK | ADC_CHAN_POT1;

	SIM->SOPT7 = SIM_SOPT7_ADC0ALTTRGEN_MASK |
				SIM_SOPT7_ADC0TRGSEL(SIM_SOPT7_ADTGSL_VAL_PIT0);

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0u].LDVAL = PIT0_MOD - 1u;
	PIT->CHANNEL[0u].TCTRL = PIT_TCTRL_TEN_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
