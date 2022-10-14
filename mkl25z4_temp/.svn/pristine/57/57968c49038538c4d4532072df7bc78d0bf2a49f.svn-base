/*
 * Name: adc_irq_trig_sw_tconv.c
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

void __attribute__ ((interrupt)) ADC0_IRQHandler(void)
{
	GPIO_J8->PCOR = IOMASK_J8;
	led_bgraph((ADC0->R[0u] >> 8u) * 0.035f);
}

void __attribute__ ((interrupt)) PIT_IRQHandler(void)
{
	PIT->CHANNEL[0u].TFLG = PIT_TFLG_TIF_MASK;
	GPIO_J8->PSOR = IOMASK_J8;
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

	GPIO_J8->PDDR |= IOMASK_J8;
	PORT_J8->PCR[IOIND_J8] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	ADC0->CFG1 = ADC_CFG1_ADIV(ADC_CFG1_ADIV_VAL_DIV8) | ADC_CFG1_ADLSMP_MASK |
					ADC_CFG1_MODE(ADC_CFG1_MODE_VAL_16BIT);
	ADC0->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(ADC_SC3_AVGS_VAL_32SAMP);

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0u].LDVAL = PIT0_MOD - 1u;
	PIT->CHANNEL[0u].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
