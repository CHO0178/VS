/*
 *==============================================================================
 * Name: asm_uart_gas.s
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global uart_init_asm
	.global uart_send_asm

	.set UART_BDH_OFFSET,		0x00
	.set UART_BDL_OFFSET,		0x01
	.set UART_C2_OFFSET,		0x03
	.set UART_S1_OFFSET,		0x04
	.set UART_D_OFFSET,			0x07

	.set UART_BDH_SBR_MASK,		0x1F
	.set UART_C2_TE_MASK,		0x08
	.set UART_S1_TDRE_MASK,		0x80

/*
 *==============================================================================
 * uart_init_asm
 *==============================================================================
 */

	.section .text.uart_init_asm, "ax", %progbits
	.type uart_init_asm, %function
	.thumb_func
uart_init_asm:

	mov r3, r1
	lsrs r1, #8
	ldr r2, =UART_BDH_SBR_MASK
	ands r1, r2
	strb r1, [r0, #UART_BDH_OFFSET]
	strb r3, [r0, #UART_BDL_OFFSET]
	ldr r1, =UART_C2_TE_MASK
	strb r1, [r0, #UART_C2_OFFSET]
	bx lr

	.size uart_init_asm, . - uart_init_asm

/*
 *==============================================================================
 * uart_send_asm
 *==============================================================================
 */

	.section .text.uart_send_asm, "ax", %progbits
	.type uart_send_asm, %function
	.thumb_func
uart_send_asm:

	ldr r2, =UART_S1_TDRE_MASK
uart_send_asm_loop:
	ldrb r3, [r0, #UART_S1_OFFSET]
	ands r3, r2
	beq uart_send_asm_loop
	strb r1, [r0, #UART_D_OFFSET]
	bx lr

	.size uart_send_asm, . - uart_send_asm

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
