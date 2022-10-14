/*
 * Name: btn.h
 * Author: Martin Stankus
 *
 */

#ifndef _BTN_H_
#define _BTN_H_

#include <stdint.h>

#define BTN_ID4						4u
#define BTN_ID3						3u
#define BTN_ID2						2u
#define BTN_ID1						1u

#define BTN_STATE_PUSHED			1u
#define BTN_STATE_RELEASED			0u

void btn_init(void);
uint8_t btn_get(uint8_t id);

#endif /* _BTN_H_ */
