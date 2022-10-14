/*
 *==============================================================================
 * Name: fib_locvar.S
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global fib_locvar

/*
 *==============================================================================
 * fib_locvar
 *==============================================================================
 */

	//there are two (32-bit) local variables: TEMP and REGSAVE
 	.set FIB_LOCVAR_TEMP_OFFSET,		0x0
 	.set FIB_LOCVAR_REGSAVE_OFFSET,		0x4

	.section .text.fib_locvar, "ax", %progbits
	.type fib_locvar, %function
	.thumb_func
fib_locvar:
	push {r7, lr}
	sub sp, #8
	mov r7, sp

	cmp r1, #2
	bls fib_locvar_error

	ldr r2, =0
	ldr r3, =1

	//push {r4}
	str r4, [r7, #FIB_LOCVAR_REGSAVE_OFFSET]
	ldr r4, =3
fib_locvar_loop:
	//push {r3}
	str r3, [r7, #FIB_LOCVAR_TEMP_OFFSET]
	adds r3, r2
	//pop {r2}
	ldr r2, [r7, #FIB_LOCVAR_TEMP_OFFSET]
	adds r4, #1
	cmp r4, r1
	bls fib_locvar_loop

	str r3, [r0]
	ldr r0, =0
	//pop {r4}
	ldr r4, [r7, #FIB_LOCVAR_REGSAVE_OFFSET]
	b fib_locvar_end

fib_locvar_error:
	ldr r0, =1
fib_locvar_end:
	add sp, #8
	pop {r7, pc}

	.size fib_locvar, . - fib_locvar

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
