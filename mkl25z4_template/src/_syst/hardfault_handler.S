/*
 *==============================================================================
 * Name: hardfault_handler.S
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *
 * "It's a trap!"
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global HardFault_Handler

/*
 *==============================================================================
 * hardfault handler
 *==============================================================================
 */

	.section .text.HardFault_Handler, "ax", %progbits
	.type HardFault_Handler, %function
	.thumb_func
HardFault_Handler:

	ldr r0, [sp, 0x18]
	bkpt
	b HardFault_Handler

	.size HardFault_Handler, . - HardFault_Handler

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
