/*
 * Name: algo_random.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "wdog.h"
#include "led.h"
#include "btn.h"

int main(void)
{
	uint8_t led_count, btn_state_old, btn_state_new;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();
	btn_init();

	led_count = 0u;
	btn_state_old = btn_get(BTN_ID1);

	while (1) {
		btn_state_new = btn_get(BTN_ID1);

		//edge detection
		if ((btn_state_old == BTN_STATE_RELEASED) && (btn_state_new == BTN_STATE_PUSHED)) {
			led_bindisp(led_count);
		}
		btn_state_old = btn_state_new;

		led_count++;

		wdog_refresh();
	}

	return 0;
}
