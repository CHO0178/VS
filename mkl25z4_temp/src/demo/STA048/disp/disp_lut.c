/*
 * Name: disp_lut.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "disp_bm.h"
#include "wait_bm.h"

#include "sin_lut.h"

#define LUT_DATA_LEN		SIN_LUT_DATA_LEN
#define LUT_DATA			SIN_LUT_DATA

const uint16_t lut[LUT_DATA_LEN] = {LUT_DATA};

int main(void)
{
	uint8_t lut_ind_a, lut_ind_b;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	disp_init();

	lut_ind_a = 0u;
	lut_ind_b = LUT_DATA_LEN / 4u;

	while (1) {

		disp_draw(lut[lut_ind_a] >> 6u, lut[lut_ind_b] >> 6u, DISP_NO_DRAW, DISP_NO_DRAW);

		lut_ind_a++;
		if (lut_ind_a == LUT_DATA_LEN) {
			lut_ind_a = 0u;
		}

		lut_ind_b++;
		if (lut_ind_b == LUT_DATA_LEN) {
			lut_ind_b = 0u;
		}

		wait_bm(WAIT_BM_20MSEC_FSYS48M, 1u);
	}

	return 0;
}
