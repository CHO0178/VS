/*
 *==============================================================================
 * Name: asm_fib_gas.s
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global fib_asm

/*
 *==============================================================================
 * fib_asm
 *==============================================================================
 */

	.section .text.fib_asm, "ax", %progbits
	.type fib_asm, %function
	.thumb_func
fib_asm:

	mov r1, r0
	ldr r0, =0
	cmp r1, #3
	blo fib_asm_end

	ldr r2, =0
	ldr r3, =1
	push {r4}

	ldr r0, =3
fib_asm_loop:
	adds r4, r3, r2
	mov r2, r3
	mov r3, r4
	adds r0, #1
	cmp r0, r1
	bls fib_asm_loop

	mov r0, r4
	pop {r4}
fib_asm_end:
	bx lr

	.size fib_asm, . - fib_asm

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
