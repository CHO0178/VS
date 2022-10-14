/*
 * Name: algo_factorial.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "wdog.h"
#include "led.h"
#include "btn.h"

int main(void)
{
	uint8_t btn_state_old, btn_state_new;
	uint16_t input, factorial;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();
	btn_init();

	input = 0u;
	btn_state_old = btn_get(BTN_ID1);

	while (btn_get(BTN_ID2) != BTN_STATE_PUSHED) {
		btn_state_new = btn_get(BTN_ID1);

		//edge detection
		if ((btn_state_old == BTN_STATE_RELEASED) && (btn_state_new == BTN_STATE_PUSHED)) {
			input++;
		}
		btn_state_old = btn_state_new;

		led_bindisp(input);

		wdog_refresh();
	}

	factorial = 1u;
	while (input > 0u) {
		factorial = factorial * input;
		input = input - 1u;
	}

	while (1) {
		if (btn_get(BTN_ID2) == BTN_STATE_PUSHED) {
			led_bindisp(factorial >> 8u);
		} else {
			led_bindisp(factorial);
		}

		wdog_refresh();
	}

	return 0;
}
