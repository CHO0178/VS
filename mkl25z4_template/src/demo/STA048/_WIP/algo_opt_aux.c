/*
 * Name: algo_opt_aux.c
 * Author: Martin Stankus
 *
 */

#include <stdint.h>
#include "algo_opt_aux.h"

uint32_t gva, gvb, wf1;
volatile uint32_t gvc,wf2;

void algo_opt_aux(volatile uint32_t *arg1, uint32_t *arg2)
{
	static uint32_t auxa;
	static volatile uint32_t auxb;

	auxa = 1;
	auxa = 1;

	auxb = 1;
	auxb = 1;

	*arg1 = 1;
	*arg1 = 1;

	*arg2 = 1;
	*arg2 = 1;

	gva = 1;
	gva = 1;

	gvb = 1;
	gvb = 1;

	gvc = 1;
	gvc = 1;

	while (wf1 == 0);

	while (wf2 == 0);
}
