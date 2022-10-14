/*
 * Name: fib_ref.c
 * Author: Martin Stankus
 *
 */

#include "fib_ref.h"

unsigned int fib_ref(unsigned int *res, unsigned int rank_lim)
{
	unsigned int rank, new, old, temp;

	if (rank_lim <= 2) {
		return 1u;
	}

	old = 0u;
	new = 1u;

	for (rank = 3u; rank <= rank_lim; rank++) {
		temp = new;
		new += old;
		old = temp;
	}

	*res = new;

	return 0u;
}
