/*
 * Name: tpm_pwm_bnc.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "adc_bm.h"

#define PWM_PERIOD		255ul

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	PORT_J12->PCR[IOIND_J12] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	TPM0->CONTROLS[TPMCH_J12].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
	TPM0->MOD = PWM_PERIOD - 1u;
	TPM0->SC = TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | TPM_SC_PS(TPM_SC_PS_VAL_D2);

	while (1) {
		TPM0->CONTROLS[TPMCH_J12].CnV = adc_bm_read(ADC_BM_MODE_8BIT_SE, ADC_CHAN_POT1);
		wdog_refresh();
	}

	return 0;
}
