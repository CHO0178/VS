/*
 * Name: dac_lut_buf.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "sin_lut.h"

#define DAC0_EXPT_PRI		2u

#define PIT0_MOD			3000ul

#define DAC0_DAT0			(*((volatile uint16_t *) 0x4003F000ul))
#define DAC0_DAT1			(*((volatile uint16_t *) 0x4003F002ul))

uint16_t sin_lut_ind;
const uint16_t sin_lut[SIN_LUT_DATA_LEN] = {SIN_LUT_DATA};

void __attribute__ ((interrupt)) DAC0_IRQHandler(void)
{
	if (DAC0->SR & DAC_SR_DACBFRPTF_MASK) {
		DAC0->SR &= ~DAC_SR_DACBFRPTF_MASK;
		DAC0_DAT1 = sin_lut[sin_lut_ind];
	}

	if (DAC0->SR & DAC_SR_DACBFRPBF_MASK) {
		DAC0->SR &= ~DAC_SR_DACBFRPBF_MASK;
		DAC0_DAT0 = sin_lut[sin_lut_ind];
	}

	sin_lut_ind++;
	if (sin_lut_ind == SIN_LUT_DATA_LEN) {
		sin_lut_ind = 0;
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	NVIC_SetPriority(DAC0_IRQn, DAC0_EXPT_PRI);
	NVIC_EnableIRQ(DAC0_IRQn);

	DAC0->C1 = DAC_C1_DACBFEN_MASK;
	DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACBTIEN_MASK | DAC_C0_DACBBIEN_MASK;

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0u].LDVAL = PIT0_MOD - 1u;
	PIT->CHANNEL[0u].TCTRL = PIT_TCTRL_TEN_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
