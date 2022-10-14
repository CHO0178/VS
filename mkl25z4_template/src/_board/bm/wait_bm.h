/*
 * Name: wait_bm.h
 * Author: Martin Stankus
 *
 * This module uses SysTick timer.
 * DO NOT use SysTick timer if wait_bm() is used.
 *
 */

#ifndef _WAIT_BM_H_
#define _WAIT_BM_H_

#include <stdint.h>

#define WAIT_BM_100USEC_FSYS48M		300ul
#define WAIT_BM_250USEC_FSYS48M		750ul
#define WAIT_BM_500USEC_FSYS48M		1500ul

#define WAIT_BM_1MSEC_FSYS48M		3000ul
#define WAIT_BM_2MSEC_FSYS48M		6000ul
#define WAIT_BM_3MSEC_FSYS48M		9000ul
#define WAIT_BM_4MSEC_FSYS48M		12000ul
#define WAIT_BM_5MSEC_FSYS48M		15000ul

#define WAIT_BM_10MSEC_FSYS48M		30000ul
#define WAIT_BM_20MSEC_FSYS48M		60000ul
#define WAIT_BM_100MSEC_FSYS48M		300000ul
#define WAIT_BM_500MSEC_FSYS48M		1500000ul
#define WAIT_BM_1000MSEC_FSYS48M	3000000ul
#define WAIT_BM_2000MSEC_FSYS48M	6000000ul
#define WAIT_BM_3000MSEC_FSYS48M	9000000ul

void wait_bm(uint32_t sys_ticks_x16, uint8_t wdog_ref_en);

#endif /* _WAIT_BM_H_ */
