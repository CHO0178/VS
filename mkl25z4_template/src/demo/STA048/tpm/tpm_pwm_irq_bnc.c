/*
 * Name: tpm_pwm_irq_bnc.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"

#define TPM0_EXPT_PRI	2u

//fpwm = (48 MHz / 2) / PWM_PERIOD = 93750 Hz
#define PWM_PERIOD		256ul

//duty cycle is increased with fincr = 50 Hz
//OVF_LIMIT = fpwm / fincr
#define OVF_LIMIT		1875ul

#define TPM_SC_COMPVAL	(TPM_SC_TOIE_MASK | \
						TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | \
						TPM_SC_PS(TPM_SC_PS_VAL_D2))

uint32_t ovf_count;
uint8_t chan_val;

void __attribute__ ((interrupt)) TPM0_IRQHandler(void)
{
	TPM0->SC = TPM_SC_TOF_MASK | TPM_SC_COMPVAL;

	ovf_count++;
	if (ovf_count == OVF_LIMIT) {
		ovf_count = 0ul;
		TPM0->CONTROLS[TPMCH_J12].CnV = chan_val++;
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	NVIC_SetPriority(TPM0_IRQn, TPM0_EXPT_PRI);
	NVIC_EnableIRQ(TPM0_IRQn);

	PORT_J12->PCR[IOIND_J12] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	TPM0->CONTROLS[TPMCH_J12].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
	TPM0->MOD = PWM_PERIOD - 1u;
	TPM0->SC = TPM_SC_COMPVAL;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
