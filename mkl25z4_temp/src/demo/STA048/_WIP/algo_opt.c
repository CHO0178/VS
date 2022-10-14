/*
 * Name: algo_opt.c
 * Author: Martin Stankus
 *
 */

#include <stdint.h>
#include "algo_opt_aux.h"

int main(void)
{
	uint32_t va, vb, *va_p, *vb_p;
	volatile uint32_t vc, *vc_p;

	va_p = &va;
	vb_p = &vb;
	vc_p = &vc;

	va = 1;
	va = 1;
	*va_p = 1;
	*va_p = 1;

	vb = 1;
	vb = 1;
	*vb_p = 1;
	*vb_p = 1;

	vc = 1;
	vc = 1;

	algo_opt_aux(va_p, va_p);

	*vc_p = 1;
	*vc_p = 1;

	return 0;
}
