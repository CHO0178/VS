/*
 * Name: irq_systick.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "wdog.h"
#include "led.h"

#define DISP_PERIOD			1500000ul

#define SYSTICK_EXPT_PRI	2u

uint8_t led_val;

void __attribute__ ((interrupt)) SysTick_Handler(void)
{
	led_bindisp(++led_val);
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();

	NVIC_SetPriority(SysTick_IRQn, SYSTICK_EXPT_PRI);

	SysTick->LOAD = DISP_PERIOD - 1u;
	SysTick->VAL = 0ul;
	SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
