/*
 * Name: algo_blinker.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "wdog.h"
#include "led.h"
#include "btn.h"

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();
	btn_init();

	while (1) {

		if (btn_get(BTN_ID4) == BTN_STATE_PUSHED) {
			led_on(LED_ID4);
		} else {
			led_off(LED_ID4);
		}

		if (btn_get(BTN_ID3) == BTN_STATE_PUSHED) {
			led_on(LED_ID3);
		} else {
			led_off(LED_ID3);
		}

		if (btn_get(BTN_ID2) == BTN_STATE_PUSHED) {
			led_on(LED_ID2);
		} else {
			led_off(LED_ID2);
		}

		if (btn_get(BTN_ID1) == BTN_STATE_PUSHED) {
			led_on(LED_ID1);
		} else {
			led_off(LED_ID1);
		}

		wdog_refresh();
	}

	return 0;
}
