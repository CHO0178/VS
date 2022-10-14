#include "MKL25Z4.h"
#include "wdog.h"
#include "soc_def.h"
#include "res_alloc.h"
#include "led.h"

//priorita preruseni
#define ADC_EXPT_PRI		2u

#define TPM_TRG_MOD			37500ul

void __attribute__ ((interrupt)) ADC0_IRQHandler(void)
{
	//proc je citlivost 0.0175f?
	led_low_bgraph(ADC0->R[0u] * 0.0175f);
	led_high_bgraph(ADC0->R[1u] * 0.0175f);
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();

	//NVIC Setup pro ADC0
	NVIC_SetPriority(ADC0_IRQn, ADC_EXPT_PRI);
	NVIC_EnableIRQ(ADC0_IRQn);

	//ADC0 Setup
	// CFG1 - delicka 4
	//??
	//SC2 - hardware trigger
	//??
	//SC1[0u] registr - vyber kanalu (potenciometer 1 - pinout SEn)
	//??
	//SC1[1u] registr - povolit preruseni + vyber kanalu (potenciometer 2 - pinout SEn)
	//??

	//SIM->SOPT7 setup not necessary, default vals are ok

	//TPM Setup
	TPM1->CONTROLS[0u].CnSC = TPM_CnSC_MSA_MASK;
	TPM1->CONTROLS[0u].CnV = 9375ul;
	TPM1->CONTROLS[1u].CnSC = TPM_CnSC_MSA_MASK;
	TPM1->CONTROLS[1u].CnV = 18750ul;
	TPM1->MOD = TPM_TRG_MOD - 1u;
	TPM1->SC = TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | TPM_SC_PS(TPM_SC_PS_VAL_D128);

	while (1) {
		wdog_refresh();
	}

	return 0;
}
