/*
 * Name: algo_fibonacci.c
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
	uint8_t fib_a, fib_b, fib_next, cnt, lim;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();
	btn_init();

	lim = 3u;
	btn_state_old = btn_get(BTN_ID1);

	while (btn_get(BTN_ID2) != BTN_STATE_PUSHED) {
		btn_state_new = btn_get(BTN_ID1);

		//edge detection
		if ((btn_state_old == BTN_STATE_RELEASED) && (btn_state_new == BTN_STATE_PUSHED)) {
			lim++;
		}
		btn_state_old = btn_state_new;

		led_bindisp(lim);

		wdog_refresh();
	}

	fib_a = 0u;
	fib_b = 1u;

	for (cnt = 3u; cnt <= lim; cnt++) {
		fib_next = fib_a + fib_b;
		fib_a = fib_b;
		fib_b = fib_next;
	}

	led_bindisp(fib_b);

	while (1) {
		wdog_refresh();
	}

	return 0;
}
