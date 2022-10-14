/*
 * Name: tsi_led.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "led.h"

#define TSI_GENCS_COMPVAL	(TSI_GENCS_ESOR_MASK | \
							TSI_GENCS_REFCHRG(7u) | \
							TSI_GENCS_DVOLT(3u) | \
							TSI_GENCS_EXTCHRG(7u) |\
							TSI_GENCS_PS(0u) |\
							TSI_GENCS_NSCN(31u) |\
							TSI_GENCS_TSIEN_MASK |\
							TSI_GENCS_TSIIEN_MASK)

void TSI0_IRQHandler(void)
{
	static uint16_t phase, val_chan9, val_chan10;

	phase++;

	if (phase == 1u) {
		//left electrode
		val_chan9 = TSI0->DATA;
		TSI0->GENCS = TSI_GENCS_COMPVAL | TSI_GENCS_EOSF_MASK;
		TSI0->DATA = TSI_DATA_TSICH(10u) | TSI_DATA_SWTS_MASK;
	} else {
		//right electrode
		val_chan10 = TSI0->DATA;

		//left
		if (val_chan9 > 85u) {
			led_rgb_set(LED_ID_RED, LED_RGB_PWM_PERIOD / 20u);
		} else {
			led_rgb_set(LED_ID_RED, 0u);
		}

		//right
		if (val_chan10 > 85u) {
			led_rgb_set(LED_ID_GREEN, LED_RGB_PWM_PERIOD / 20u);
		} else {
			led_rgb_set(LED_ID_GREEN, 0u);
		}

		TSI0->GENCS = TSI_GENCS_COMPVAL | TSI_GENCS_EOSF_MASK;
		TSI0->DATA = TSI_DATA_TSICH(9u) | TSI_DATA_SWTS_MASK;

		phase = 0u;
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_rgb_init();

	NVIC_SetPriority(TSI0_IRQn, 0u);
	NVIC_EnableIRQ(TSI0_IRQn);

	TSI0->GENCS = TSI_GENCS_COMPVAL;
	TSI0->DATA = TSI_DATA_TSICH(9u) | TSI_DATA_SWTS_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
