/*
 *==============================================================================
 * Name: nmi_handler.S
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global NMI_Handler

/*
 *==============================================================================
 * nmi handler
 *==============================================================================
 */

	.section .text.NMI_Handler, "ax", %progbits
	.type NMI_Handler, %function
	.thumb_func
NMI_Handler:

	bx lr

	.size NMI_Handler, . - NMI_Handler

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
