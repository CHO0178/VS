/*
 *==============================================================================
 * Name: mem.S
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global memcmp
	.global memcpy
	.global memmove
	.global memset

/*
 *==============================================================================
 * memcmp
 *==============================================================================
 */

	.section .text.memcmp, "ax", %progbits
	.type memcmp, %function
	.thumb_func
memcmp:

	ldr r3, =0
	cmp r2, #0
	beq memcmp_end
	push {r4-r5}
memcmp_loop:
	ldrb r4, [r0, r3]
	ldrb r5, [r1, r3]
	adds r3, #1
	cmp r4, r5
	beq memcmp_eq

	bhi memcmp_noteq
	rsbs r3, r3, #0
	b memcmp_noteq

memcmp_eq:
	cmp r3, r2
	bne memcmp_loop
	ldr r3, =0
memcmp_noteq:
	pop {r4-r5}
memcmp_end:
	mov r0, r3
	bx lr

	.size memcmp, . - memcmp

/*
 *==============================================================================
 * memcpy
 *==============================================================================
 */

	.section .text.memcpy, "ax", %progbits
	.type memcpy, %function
	.thumb_func
memcpy:

	cmp r2, #0
	beq memcpy_end
memcpy_loop:
	subs r2, #1
	ldrb r3, [r1, r2]
	strb r3, [r0, r2]
	bne memcpy_loop
memcpy_end:
	bx lr

	.size memcpy, . - memcpy

/*
 *==============================================================================
 * memmove
 *==============================================================================
 */

	.section .text.memmove, "ax", %progbits
	.type memmove, %function
	.thumb_func
memmove:

	push {lr}
	cmp r1, r0
	bhi memmove_alt
	bl memcpy
	b memmove_end
memmove_alt:
	bl memcpy_up
memmove_end:
	pop {pc}

	.size memmove, . - memmove

/*
 *==============================================================================
 * memset
 *==============================================================================
 */

	.section .text.memset, "ax", %progbits
	.type memset, %function
	.thumb_func
memset:

	cmp r2, #0
	beq memset_end
memset_loop:
	subs r2, #1
	strb r1, [r0, r2]
	bne memset_loop
memset_end:
	bx lr

	.size memset, . - memset

/*
 *==============================================================================
 * memcpy_up
 *==============================================================================
 */

	.section .text.memcpy_up, "ax", %progbits
	.type memcpy_up, %function
	.thumb_func
memcpy_up:

	cmp r2, #0
	beq memcpy_up_end
	ldr r3, =0
	push {r4}
memcpy_up_loop:
	ldrb r4, [r1, r3]
	strb r4, [r0, r3]
	adds r3, #1
	cmp r3, r2
	bne memcpy_up_loop
	pop {r4}
memcpy_up_end:
	bx lr

	.size memcpy_up, . - memcpy_up

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
