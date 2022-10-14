/*
 * Name: wdog.h
 * Author: Martin Stankus
 *
 */

#ifndef _WDOG_H_
#define _WDOG_H_

typedef enum {
	WDOG_CONF_DIS,
	WDOG_CONF_LPOCLK_32_CYCLES,
	WDOG_CONF_LPOCLK_256_CYCLES,
	WDOG_CONF_LPOCLK_1024_CYCLES,
	WDOG_CONF_BUSCLK_8192_CYCLES,
	WDOG_CONF_BUSCLK_65536_CYCLES,
	WDOG_CONF_BUSCLK_262144_CYCLES
} WDOG_CONF;

void wdog_init(WDOG_CONF wdog_conf);
void wdog_refresh(void);

#endif /* _WDOG_H_ */
