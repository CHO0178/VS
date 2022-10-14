/*
 * Name: tpm_pwm_led.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "adc_bm.h"

//fpwm = (48 MHz / 2) / PWM_PERIOD = 94118 Hz
#define PWM_PERIOD		255ul

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	PORT_LEDR->PCR[IOIND_LEDR] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	TPM2->CONTROLS[TPMCH_LEDR].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	TPM2->MOD = PWM_PERIOD - 1u;
	TPM2->SC = TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | TPM_SC_PS(TPM_SC_PS_VAL_D2);

	while (1) {
		TPM2->CONTROLS[TPMCH_LEDR].CnV = adc_bm_read(ADC_BM_MODE_8BIT_SE, ADC_CHAN_POT1);
		wdog_refresh();
	}

	return 0;
}
