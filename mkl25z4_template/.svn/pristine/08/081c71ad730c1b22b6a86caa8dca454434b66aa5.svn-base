/*
 * Name: dac_pwm_lut.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "sin_lut.h"

#define PIT_EXPT_PRI		2u

#define PIT0_MOD			120000ul

#define DAC0_DAT0			(*((volatile uint16_t *) 0x4003F000ul))

uint16_t sin_lut_ind;
const uint16_t sin_lut[SIN_LUT_DATA_LEN] = {SIN_LUT_DATA};

void __attribute__ ((interrupt)) PIT_IRQHandler(void)
{
	PIT->CHANNEL[0u].TFLG = PIT_TFLG_TIF_MASK;

	DAC0_DAT0 = sin_lut[sin_lut_ind];
	TPM_J12->CONTROLS[TPMCH_J12].CnV = sin_lut[sin_lut_ind];

	sin_lut_ind++;
	if (sin_lut_ind == SIN_LUT_DATA_LEN) {
		sin_lut_ind = 0;
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	DAC0->C0 = DAC_C0_DACEN_MASK;

	PORT_J12->PCR[IOIND_J12] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	TPM_J12->CONTROLS[TPMCH_J12].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
	TPM_J12->MOD = 4094u;
	TPM_J12->CONF = TPM_CONF_DBGMODE(TPM_CONF_DBGM_VAL_RUN);
	TPM_J12->SC = TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | TPM_SC_PS(TPM_SC_PS_VAL_D1);

	NVIC_SetPriority(PIT_IRQn, PIT_EXPT_PRI);
	NVIC_EnableIRQ(PIT_IRQn);

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0u].LDVAL = PIT0_MOD - 1u;
	PIT->CHANNEL[0u].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
