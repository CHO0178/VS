/*
 * Name: bsofr_ref.c
 * Author: Martin Stankus
 *
 */

#include "bsort_ref.h"

void bsort_ref(int data[], int len)
{
	int swap;

	if (len < 2) {
		return;
	}

	for (int ind_lim = len - 2; ind_lim >= 0; ind_lim--) {
		for (int ind = 0; ind <= ind_lim; ind++) {

			if (data[ind + 1] < data[ind]) {
				swap = data[ind + 1];
				data[ind + 1] = data[ind];
				data[ind] = swap;
			}

		}
	}
}
