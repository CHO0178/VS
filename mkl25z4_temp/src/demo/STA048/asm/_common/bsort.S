/*
 *==============================================================================
 * Name: bsort.S
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global bsort

/*
 *==============================================================================
 * bsort
 *==============================================================================
 */

	.section .text.bsort, "ax", %progbits
	.type bsort, %function
	.thumb_func
bsort:
	cmp r1, #2
	blt bsort_end

	push {r4-r6}

	subs r1, #2
bsort_loop_ind_lim:

	eors r2, r2
bsort_loop_ind:

	lsls r6, r2, #2
	mov r3, r6
	adds r3, #4
	ldr r4, [r0, r6]
	ldr r5, [r0, r3]
	cmp r5, r4
	bge bsort_no_swap

	str r4, [r0, r3]
	str r5, [r0, r6]

bsort_no_swap:

	adds r2, #1
	cmp r2, r1
	ble bsort_loop_ind

	subs r1, #1
	bge bsort_loop_ind_lim

	pop {r4-r6}

bsort_end:
	bx lr

	.size bsort, . - bsort

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
