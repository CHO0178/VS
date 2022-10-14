/*
 * Name: main.c
 * Author: You
 *
 * This is a stub. Write your code here.
 *
 */

#include "wdog.h"

__attribute__ ((weak)) int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	while (1) {
		wdog_refresh();
	}

	return 0;
}
