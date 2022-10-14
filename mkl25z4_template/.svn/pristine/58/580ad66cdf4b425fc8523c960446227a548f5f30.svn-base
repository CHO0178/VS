/*
 *==============================================================================
 * Name: asm_glob_gas.s
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global glob_var
	.global glob_var_dummy
	.global glob_set_asm
	.global glob_get_asm

/*
 *==============================================================================
 * global variables
 *==============================================================================
 */

	.section .data, "aw", %progbits

	.type glob_var, %object
glob_var:
	.long 0xBEEFFEED
	.size glob_var, . - glob_var

	.type glob_var_dummy, %object
glob_var_dummy:
	.long 0xDEADCAFE
	.size glob_var_dummy, . - glob_var_dummy

/*
 *==============================================================================
 * glob_set_asm
 *==============================================================================
 */

	.section .text.glob_set_asm, "ax", %progbits
	.type glob_set_asm, %function
	.thumb_func
glob_set_asm:

	ldr r1, =glob_var
	str r0, [r1]
	bx lr

	.size glob_set_asm, . - glob_set_asm

 /*
 *==============================================================================
 * glob_get_asm
 *==============================================================================
 */

	.section .text.glob_get_asm, "ax", %progbits
	.type glob_get_asm, %function
	.thumb_func
glob_get_asm:

	ldr r1, =glob_var
	ldr r0, [r1]
	bx lr

	.size glob_get_asm, . - glob_get_asm

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
