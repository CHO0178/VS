/*
 *==============================================================================
 * Name: bsort_fptr.S
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global bsort_fptr

/*
 *==============================================================================
 * bsort_fptr
 *==============================================================================
 */

	.section .text.bsort_fptr, "ax", %progbits
	.type bsort_fptr, %function
	.thumb_func
bsort_fptr:
	push {lr}

	cmp r1, #2
	blt bsort_fptr_end

	push {r4-r7}

	subs r1, #2
bsort_fptr_loop_ind_lim:

	eors r3, r3
bsort_fptr_loop_ind:

	lsls r7, r3, #2
	mov r4, r7
	adds r4, #4
	ldr r5, [r0, r7]
	ldr r6, [r0, r4]

	push {r0-r3}
	mov r0, r5
	mov r1, r6
	blx r2
	cmp r0, #0
	pop {r0-r3}

	beq bsort_fptr_no_swap

	str r5, [r0, r4]
	str r6, [r0, r7]

bsort_fptr_no_swap:

	adds r3, #1
	cmp r3, r1
	ble bsort_fptr_loop_ind

	subs r1, #1
	bge bsort_fptr_loop_ind_lim

	pop {r4-r7}

bsort_fptr_end:
	pop {pc}

	.size bsort_fptr, . - bsort_fptr

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
