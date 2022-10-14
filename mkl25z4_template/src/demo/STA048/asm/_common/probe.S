/*
 *==============================================================================
 * Name: probe.S
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global probe_hardfault_handler
	.global probe

/*
 *==============================================================================
 * probe_hardfault_handler
 *==============================================================================
 */

	.section .text.probe_hardfault_handler, "ax", %progbits
	.type probe_hardfault_handler, %function
	.thumb_func
probe_hardfault_handler:

	ldr r0, [sp, 0x18]
	adds r0, #2
	str r0, [sp, 0x18]
	eors r0, r0
	str r0, [sp]
	bx lr

	.size probe_hardfault_handler, . - probe_hardfault_handler

/*
 *==============================================================================
 * probe
 *==============================================================================
 */

	.section .text.probe, "ax", %progbits
	.type probe, %function
	.thumb_func
probe:

	mov r2, r0
	ldr r0, =1
	ldrb r1, [r1]
	strb r1, [r2]
	bx lr

	.size probe, . - probe

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
