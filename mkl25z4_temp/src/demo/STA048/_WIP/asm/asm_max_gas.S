/*
 *==============================================================================
 * Name: asm_max_gas.s
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global max_asm

/*
 *==============================================================================
 * max_asm
 *==============================================================================
 */

	.section .text.max_asm, "ax", %progbits
	.type max_asm, %function
	.thumb_func
max_asm:

	mov r2, r0
	ldr r0, =0
	cmp r1, #0
	beq asm_max_end

	ldrh r0, [r2]
	push {r4}

	ldr r3, =0
	adds r1, r1
max_asm_loop:
	ldrh r4, [r2, r3]
	cmp r4, r0
	bls max_asm_not_higher
	mov r0, r4
max_asm_not_higher:
	adds r3, #2
	cmp r3, r1
	blo max_asm_loop

	pop {r4}
asm_max_end:
	bx lr

	.size max_asm, . - max_asm

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
