/*
 * Name: tpm_pwm_led_adctrg.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "led.h"

#define ADC_EXPT_PRI			2u

//TPM1CH0 triggers ADC[0], TPM1CH1 triggers ADC[1]
//fvz = (48 MHz / 128) / ADC_MEAS_PERIOD = 10 Hz
//0.25s meas offset for adc[0], 0.5s meas offset for adc[1]
#define ADC_MEAS_PERIOD			37500ul
#define ADC_CH0_MEAS_OFFSET		9375ul
#define ADC_CH1_MEAS_OFFSET		18750ul

//fpwm = (48 MHz / 2) / PWM_PERIOD = 94118 Hz
#define PWM_PERIOD				255ul

void __attribute__ ((interrupt)) ADC0_IRQHandler(void)
{
	TPM2->CONTROLS[TPMCH_LEDR].CnV = ADC0->R[0u];
	TPM2->CONTROLS[TPMCH_LEDG].CnV = ADC0->R[1u];
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();

	PORT_LEDR->PCR[IOIND_LEDR] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_LEDG->PCR[IOIND_LEDG] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	TPM2->CONTROLS[TPMCH_LEDR].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	TPM2->CONTROLS[TPMCH_LEDG].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	TPM2->MOD = PWM_PERIOD - 1u;
	TPM2->SC = TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | TPM_SC_PS(TPM_SC_PS_VAL_D2);

	NVIC_SetPriority(ADC0_IRQn, ADC_EXPT_PRI);
	NVIC_EnableIRQ(ADC0_IRQn);

	ADC0->CFG1 = ADC_CFG1_ADIV(ADC_CFG1_ADIV_VAL_DIV4);
	ADC0->SC2 = ADC_SC2_ADTRG_MASK;
	ADC0->SC1[0u] = ADC_CHAN_POT1;
	ADC0->SC1[1u] = ADC_SC1_AIEN_MASK | ADC_CHAN_POT2;

	//SIM->SOPT7 setup not necessary, default vals are ok

	TPM1->CONTROLS[0u].CnSC = TPM_CnSC_MSA_MASK;
	TPM1->CONTROLS[0u].CnV = ADC_CH0_MEAS_OFFSET;
	TPM1->CONTROLS[1u].CnSC = TPM_CnSC_MSA_MASK;
	TPM1->CONTROLS[1u].CnV = ADC_CH1_MEAS_OFFSET;
	TPM1->MOD = ADC_MEAS_PERIOD - 1u;
	TPM1->SC = TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | TPM_SC_PS(TPM_SC_PS_VAL_D128);

	while (1) {
		wdog_refresh();
	}

	return 0;
}
