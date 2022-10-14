/*
 * Name: asm_fib.h
 * Author: Martin Stankus
 *
 */

#ifndef _ASM_FIB_H_
#define _ASM_FIB_H_

#include <stdint.h>

uint32_t fib_c(uint16_t limit);
uint32_t fib_asm(uint16_t limit);

#endif /* _ASM_FIB_H_ */
