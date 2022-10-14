#include "MKL25Z4.h"
#include "wdog.h"
#include "soc_def.h"
#include "res_alloc.h"
#include "led.h"

//napiste makro bez parametru pro prioritu preruseni ADC0 - ADC_EXPT_PRI rovna 2
//??

#define PIT0_MOD		240000ul

//obsluha preruseni
void __attribute__ ((interrupt)) ADC0_IRQHandler(void)
{
	led_bgraph(ADC0->R[0u] * 0.035f);
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();

	//NVIC setup ADC0
	//adresa a priorita preruseni
	//??
	//povoleni preruseni
	//??

	//ADC0 Setup
	//CFG1 registr - delicka 4
	//??
	//SC2 registr - Hardware trigger selected
	//??
	//SC1[0u] registr - vyber kanalu (potenciometer 1 - pinout SEn) + povolte preruseni
	//??

	//SIM modul pro vyber triggeru - PIT trigger - kdyz PIT docita spusti konverzi
	SIM->SOPT7 = SIM_SOPT7_ADC0ALTTRGEN_MASK |
				SIM_SOPT7_ADC0TRGSEL(SIM_SOPT7_ADTGSL_VAL_PIT0);

	//PIT Setup
	//MCR registr - timer necita v debug modu
	//??
	// LDVAL (Channel 0u) - zapiste modulo hodnotu
	//??
	//TCTRL (Channel 0u) - timer enable
	//??

	while (1) {
		wdog_refresh();
	}

	return 0;
}
