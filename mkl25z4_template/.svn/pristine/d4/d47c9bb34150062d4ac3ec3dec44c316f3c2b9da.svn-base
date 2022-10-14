/*
 * Name: rtt_stage.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"
#include "wdog.h"

#include "SEGGER_RTT.h"

int main(void)
{
	wdog_set(WDOG_CONF_LPOCLK_1024_CYCLES);

	SEGGER_RTT_Init();
	SEGGER_RTT_WriteString(0, "Hello world!\n");

	while (1) {
		wdog_refresh();
	}

	return 0;
}
