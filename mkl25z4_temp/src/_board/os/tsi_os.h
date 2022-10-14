/*
 * Name: tsi_os.h
 * Author: Martin Stankus
 *
 */

#ifndef _TSI_OS_H_
#define _TSI_OS_H_

#include <stdint.h>

#define TSI_OS_MASK_LEFT	1u
#define TSI_OS_MASK_RIGHT	2u

void tsi_os_init(uint8_t expt_pri);
uint8_t tsi_os_read(uint16_t thr);

#endif /* _TSI_OS_H_ */
