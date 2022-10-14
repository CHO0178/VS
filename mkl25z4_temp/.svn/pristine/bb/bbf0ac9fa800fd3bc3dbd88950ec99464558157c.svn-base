/*
 * Name: asm_max_gcc.c
 * Author: Martin Stankus
 *
 */

#include <stdint.h>
#include "asm_max.h"

uint16_t max_c(uint16_t data[], uint32_t data_len)
{
	uint32_t ind;
	uint16_t max;

	if (data_len == 0) {
		return 0;
	}

	max = data[0];

	for (ind = 0; ind < data_len; ind++) {
		if (data[ind] > max) {
			max = data[ind];
		}
	}

	return max;
}
