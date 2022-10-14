/*
 *==============================================================================
 * Name: startup_MKL25Z4_vectab_ram.S
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global Reset_Handler
	.global Default_Handler

	.set SCB_BASE,			0xE000ED00
	.set SCB_VTOR_OFFSET,	0x08

/*
 *==============================================================================
 * reset handler
 *==============================================================================
 */

	.section .text.Reset_Handler, "ax", %progbits
	.type Reset_Handler, %function
	.thumb_func
Reset_Handler:

	bl SystemInit

	/* ram vectab init */

	ldr r0, =VECTAB_RAM_BEGIN
	ldr r1, =VECTAB_ROM_BEGIN
	ldr r2, =VECTAB_RAM_END
	subs r2, r0
	bl memcpy

	ldr r0, =VECTAB_RAM_BEGIN
	ldr r1, =SCB_BASE
	str r0, [r1, #SCB_VTOR_OFFSET]

	/* data section init */

	ldr r0, =DATA_BEGIN
	ldr r1, =DATA_INIT
	ldr r2, =DATA_END
	subs r2, r0
	bl memcpy

	/* bss section zeroization */

	ldr r0, =BSS_BEGIN
	ldr r1, =0
	ldr r2, =BSS_END
	subs r2, r0
	bl memset

	/* stack pattern fill */
	/* this is for debugging purposes */

	ldr r0, =STACK_BOTTOM
	ldr r1, =0xA5
	ldr r2, =STACK_TOP
	subs r2, r0
	bl memset

	bl SystemCoreClockUpdate

	bl main

stop:
	b stop

	.size Reset_Handler, . - Reset_Handler

/*
 *==============================================================================
 * default handler
 *==============================================================================
 */

	.section .text.Default_Handler, "ax", %progbits
	.type Default_Handler, %function
	.thumb_func
Default_Handler:

	bkpt
	b Default_Handler

	.size Default_Handler, . - Default_Handler

/*
 *==============================================================================
 * vector table
 *==============================================================================
 */

	.section .vectab_rom, "a", %progbits

	/* stack init value, system exceptions */
	.long STACK_TOP
	.long Reset_Handler
	.long NMI_Handler
	.long HardFault_Handler
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long 0
	.long SVCall_Handler
	.long 0
	.long 0
	.long PendSV_Handler
	.long SysTick_Handler

	/* interrupts */
	.long DMA0_IRQHandler
	.long DMA1_IRQHandler
	.long DMA2_IRQHandler
	.long DMA3_IRQHandler
	.long Reserved20_IRQHandler
	.long FTFA_IRQHandler
	.long LVD_LVW_IRQHandler
	.long LLWU_IRQHandler
	.long I2C0_IRQHandler
	.long I2C1_IRQHandler
	.long SPI0_IRQHandler
	.long SPI1_IRQHandler
	.long UART0_IRQHandler
	.long UART1_IRQHandler
	.long UART2_IRQHandler
	.long ADC0_IRQHandler
	.long CMP0_IRQHandler
	.long TPM0_IRQHandler
	.long TPM1_IRQHandler
	.long TPM2_IRQHandler
	.long RTC_IRQHandler
	.long RTC_Seconds_IRQHandler
	.long PIT_IRQHandler
	.long Reserved39_IRQHandler
	.long USB0_IRQHandler
	.long DAC0_IRQHandler
	.long TSI0_IRQHandler
	.long MCG_IRQHandler
	.long LPTMR0_IRQHandler
	.long Reserved45_IRQHandler
	.long PORTA_IRQHandler
	.long PORTD_IRQHandler

	/* vector table ends here */

	/* vectors without user handlers point to default handler */
	/* handlers are declared weak to make this possible */

	.weak NMI_Handler
	.weak HardFault_Handler
	.weak SVCall_Handler
	.weak PendSV_Handler
	.weak SysTick_Handler
	.weak DMA0_IRQHandler
	.weak DMA1_IRQHandler
	.weak DMA2_IRQHandler
	.weak DMA3_IRQHandler
	.weak Reserved20_IRQHandler
	.weak FTFA_IRQHandler
	.weak LVD_LVW_IRQHandler
	.weak LLWU_IRQHandler
	.weak I2C0_IRQHandler
	.weak I2C1_IRQHandler
	.weak SPI0_IRQHandler
	.weak SPI1_IRQHandler
	.weak UART0_IRQHandler
	.weak UART1_IRQHandler
	.weak UART2_IRQHandler
	.weak ADC0_IRQHandler
	.weak CMP0_IRQHandler
	.weak TPM0_IRQHandler
	.weak TPM1_IRQHandler
	.weak TPM2_IRQHandler
	.weak RTC_IRQHandler
	.weak RTC_Seconds_IRQHandler
	.weak PIT_IRQHandler
	.weak Reserved39_IRQHandler
	.weak USB0_IRQHandler
	.weak DAC0_IRQHandler
	.weak TSI0_IRQHandler
	.weak MCG_IRQHandler
	.weak LPTMR0_IRQHandler
	.weak Reserved45_IRQHandler
	.weak PORTA_IRQHandler
	.weak PORTD_IRQHandler

	.set NMI_Handler, Default_Handler
	.set HardFault_Handler, Default_Handler
	.set SVCall_Handler, Default_Handler
	.set PendSV_Handler, Default_Handler
	.set SysTick_Handler, Default_Handler
	.set DMA0_IRQHandler, Default_Handler
	.set DMA1_IRQHandler, Default_Handler
	.set DMA2_IRQHandler, Default_Handler
	.set DMA3_IRQHandler, Default_Handler
	.set Reserved20_IRQHandler, Default_Handler
	.set FTFA_IRQHandler, Default_Handler
	.set LVD_LVW_IRQHandler, Default_Handler
	.set LLWU_IRQHandler, Default_Handler
	.set I2C0_IRQHandler, Default_Handler
	.set I2C1_IRQHandler, Default_Handler
	.set SPI0_IRQHandler, Default_Handler
	.set SPI1_IRQHandler, Default_Handler
	.set UART0_IRQHandler, Default_Handler
	.set UART1_IRQHandler, Default_Handler
	.set UART2_IRQHandler, Default_Handler
	.set ADC0_IRQHandler, Default_Handler
	.set CMP0_IRQHandler, Default_Handler
	.set TPM0_IRQHandler, Default_Handler
	.set TPM1_IRQHandler, Default_Handler
	.set TPM2_IRQHandler, Default_Handler
	.set RTC_IRQHandler, Default_Handler
	.set RTC_Seconds_IRQHandler, Default_Handler
	.set PIT_IRQHandler, Default_Handler
	.set Reserved39_IRQHandler, Default_Handler
	.set USB0_IRQHandler, Default_Handler
	.set DAC0_IRQHandler, Default_Handler
	.set TSI0_IRQHandler, Default_Handler
	.set MCG_IRQHandler, Default_Handler
	.set LPTMR0_IRQHandler, Default_Handler
	.set Reserved45_IRQHandler, Default_Handler
	.set PORTA_IRQHandler, Default_Handler
	.set PORTD_IRQHandler, Default_Handler

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
