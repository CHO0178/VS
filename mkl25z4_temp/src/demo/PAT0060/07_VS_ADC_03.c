#include "MKL25Z4.h"
#include "wdog.h"
#include "soc_def.h"
#include "res_alloc.h"
#include "led.h"

//priority preruseni
#define ADC_EXPT_PRI		2u
#define PIT_EXPT_PRI		2u

#define PIT0_MOD			240000ul

//obsluha preruseni ADC0
void __attribute__ ((interrupt)) ADC0_IRQHandler(void)
{
	led_bgraph(ADC0->R[0u] * 0.035f);
}

//obsluha preruseni PIT
void __attribute__ ((interrupt)) PIT_IRQHandler(void)
{
	//vymazte priznak preruseni PIT - channel 0 registr TFLG
	//??
	//ADC0 - SC1[0u] - povoleno preruseni + vyber kanalu (pot1)
	//??
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();

	//NVIC setup pro ADC0 preruseni - adresa, priorita, povoleni
	NVIC_SetPriority(ADC0_IRQn, ADC_EXPT_PRI);
	NVIC_EnableIRQ(ADC0_IRQn);

	//NVIC setup pro PIT preruseni - adresa, priorita, povoleni
	NVIC_SetPriority(PIT_IRQn, PIT_EXPT_PRI);
	NVIC_EnableIRQ(PIT_IRQn);

	//ADC0 - CFG1 - nastavte delicku na 4
	//??

	//PIT setup
	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0].LDVAL = PIT0_MOD - 1u;
	PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;	// povoleni preruseni + povoleni timeru

	while (1) {
		wdog_refresh();
	}

	return 0;
}
