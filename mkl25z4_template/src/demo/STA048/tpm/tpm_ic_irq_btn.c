/*
 * Name: tpm_ic_irq_btn.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "wdog.h"

void __attribute__ ((interrupt)) TPM0_IRQHandler(void)
{
	static uint32_t time, ovf_count;

	if (TPM0->STATUS & TPM_STATUS_TOF_MASK) {
		//clear the overflow flag
		TPM0->STATUS = TPM_STATUS_TOF_MASK;

		time += 65536ul;
		ovf_count++;
	} else {
		//clear the channel flag, disable the channel (it is no longer needed)
		TPM0->CONTROLS[2].CnSC = TPM_CnSC_CHF_MASK;

		time += TPM0->CONTROLS[1u].CnV;

		//examine the variables (time and ovf_count)
		__BKPT();
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	NVIC_SetPriority(TPM0_IRQn, 0u);
	NVIC_EnableIRQ(TPM0_IRQn);

	//select OSCERCLK (8 MHz) as a clock source (the default is MCGPLLCLK/2)
	SIM->SOPT2 = (SIM->SOPT2 & ~SIM_SOPT2_TPMSRC_MASK) | SIM_SOPT2_TPMSRC(2u);

	PORTA->PCR[4u] = PORT_PCR_MUX(3u);

	//enable the channel irq, select an input capture (falling edge) mode
	TPM0->CONTROLS[1].CnSC = TPM_CnSC_CHIE_MASK | TPM_CnSC_ELSB_MASK;

	//enable the overflow irq, set the to prescaler to 8 (1 tick = 1 us)
	TPM0->SC = TPM_SC_TOIE_MASK | TPM_SC_CMOD(1u) | TPM_SC_PS(3u);

	while (1) {
		wdog_refresh();
	}

	return 0;
}
