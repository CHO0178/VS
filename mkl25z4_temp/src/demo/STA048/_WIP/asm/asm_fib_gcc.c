/*
 * Name: asm_fib_gcc.c
 * Author: Martin Stankus
 *
 */

#include <stdint.h>
#include "asm_fib.h"

uint32_t fib_c(uint16_t limit)
{
	uint32_t a, b, c, ind;

	if (limit < 3) {
		return 0;
	}

	a = 0;
	b = 1;

	for (ind = 3; ind <= limit; ind++) {
		c = b + a;
		a = b;
		b = c;
	}

	return c;
}
