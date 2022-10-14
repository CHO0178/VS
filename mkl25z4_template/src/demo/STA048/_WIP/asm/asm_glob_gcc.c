/*
 * Name: asm_glob_gcc.c
 * Author: Martin Stankus
 *
 */

#include <stdint.h>
#include "asm_glob.h"

extern uint32_t glob_var;

void glob_set_c(uint32_t val)
{
	glob_var = val;
}

uint32_t glob_get_c(void)
{
	return glob_var;
}
