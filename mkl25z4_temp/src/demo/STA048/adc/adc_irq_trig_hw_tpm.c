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

#define TPM_TRG_MOD			37500ul
#define TPM_TRG_C0V			9375ul
#define TPM_TRG_C1V			18750ul

void __attribute__ ((interrupt)) ADC0_IRQHandler(void)
{
	led_low_bgraph(ADC0->R[0u] * 0.0175f);
	led_high_bgraph(ADC0->R[1u] * 0.0175f);
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();

	NVIC_SetPriority(ADC0_IRQn, ADC_EXPT_PRI);
	NVIC_EnableIRQ(ADC0_IRQn);

	ADC0->CFG1 = ADC_CFG1_ADIV(ADC_CFG1_ADIV_VAL_DIV4);
	ADC0->SC2 = ADC_SC2_ADTRG_MASK;
	ADC0->SC1[0u] = ADC_CHAN_POT1;
	ADC0->SC1[1u] = ADC_SC1_AIEN_MASK | ADC_CHAN_POT2;

	//SIM->SOPT7 setup not necessary, default vals are ok

	TPM1->CONTROLS[0u].CnSC = TPM_CnSC_MSA_MASK;
	TPM1->CONTROLS[0u].CnV = TPM_TRG_C0V;
	TPM1->CONTROLS[1u].CnSC = TPM_CnSC_MSA_MASK;
	TPM1->CONTROLS[1u].CnV = TPM_TRG_C1V;
	TPM1->MOD = TPM_TRG_MOD - 1u;
	TPM1->SC = TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | TPM_SC_PS(TPM_SC_PS_VAL_D128);

	while (1) {
		wdog_refresh();
	}

	return 0;
}
