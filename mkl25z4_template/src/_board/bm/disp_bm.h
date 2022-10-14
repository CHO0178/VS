/*
 * Name: disp_bm.h
 * Author: Martin Stankus
 *
 */

#ifndef _DISP_BM_H_
#define _DISP_BM_H_

#include <stdint.h>

#define DISP_PAGE_CNT			8u
#define DISP_COLUMN_CNT			128u
#define DISP_PIX_PER_PAGE		(DISP_PAGE_CNT * 8u)

#define DISP_NO_DRAW			DISP_PIX_PER_PAGE

void disp_init(void);
void disp_draw(uint8_t y0, uint8_t y1, uint8_t y2, uint8_t y3);

#endif /* _DISP_BM_H_ */
