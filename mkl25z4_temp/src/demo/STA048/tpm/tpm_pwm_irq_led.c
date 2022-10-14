/*
 * Name: tpm_pwm_irq_led.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"

#define TPM2_EXPT_PRI	2u

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

void __attribute__ ((interrupt)) TPM2_IRQHandler(void)
{
	TPM2->SC = TPM_SC_TOF_MASK | TPM_SC_COMPVAL;

	ovf_count++;
	if (ovf_count == OVF_LIMIT) {
		ovf_count = 0ul;
		TPM2->CONTROLS[TPMCH_LEDR].CnV = chan_val++;
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	NVIC_SetPriority(TPM2_IRQn, TPM2_EXPT_PRI);
	NVIC_EnableIRQ(TPM2_IRQn);

	PORT_LEDR->PCR[IOIND_LEDR] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	TPM2->CONTROLS[TPMCH_LEDR].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	TPM2->MOD = PWM_PERIOD - 1u;
	TPM2->SC = TPM_SC_COMPVAL;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
