#include "MKL25Z4.h"
#include "wdog.h"
#include "led.h"
#include "soc_def.h"

#define DISP_PERIOD		15625ul

int main(void)
{
	uint8_t led_val = 0u;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();

	LPTMR0->CSR = 0ul;	//reset timeru + timer disabled

	LPTMR0->PSR = LPTMR_PSR_PRESCALE(LPTMR_PSR_PRS_VAL_D256_F128) |
					LPTMR_PSR_PCS(LPTMR_PSR_PCS_VAL_OSCERCLK);	//preskalovani
	LPTMR0->CMR = DISP_PERIOD - 1u;	//modulo hodnota
	LPTMR0->CSR = LPTMR_CSR_TEN_MASK;	//timer enable

	while (1) {
		//if CNR hodnota rovna CMR
		if (LPTMR0->CSR & LPTMR_CSR_TCF_MASK) {
			//smazu compare flag + timer enable
			LPTMR0->CSR = LPTMR_CSR_TCF_MASK | LPTMR_CSR_TEN_MASK;
			led_bindisp(++led_val);
		}

		wdog_refresh();
	}

	return 0;
}
