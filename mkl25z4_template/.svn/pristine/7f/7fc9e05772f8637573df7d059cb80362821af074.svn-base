/*
 * Name: tmr_systick.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "wdog.h"
#include "led.h"

#define DISP_PERIOD		1500000ul

int main(void)
{
	uint8_t led_val = 0u;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();

	SysTick->LOAD = DISP_PERIOD - 1u;
	SysTick->VAL = 0ul;
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;

	while (1) {
		if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
			led_bindisp(++led_val);
		}

		wdog_refresh();
	}

	return 0;
}
