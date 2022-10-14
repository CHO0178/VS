/*
 *==============================================================================
 * Name: fib.S
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global fib

/*
 *==============================================================================
 * fib
 *==============================================================================
 */

	.section .text.fib, "ax", %progbits
	.type fib, %function
	.thumb_func
fib:
	//'push {lr}' is not used if 'bx lr' is used to terminate the function
	push {lr}

	cmp r1, #2
	bls fib_error

	ldr r2, =0
	ldr r3, =1

	push {r4}
	ldr r4, =3
fib_loop:
	push {r3}
	adds r3, r2
	pop {r2}
	adds r4, #1
	cmp r4, r1
	bls fib_loop

	str r3, [r0]
	ldr r0, =0
	pop {r4}
	b fib_end

fib_error:
	ldr r0, =1
fib_end:
	//bx lr
	pop {pc}

	.size fib, . - fib

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
