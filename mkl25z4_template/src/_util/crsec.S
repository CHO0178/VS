/*
 *==============================================================================
 * Name: crsec.S
 * Author: Martin Stankus
 *
 * as docs: https://sourceware.org/binutils/docs/as/index.html
 *==============================================================================
 */

	.syntax unified
	.thumb

	.global crsec_enter
	.global crsec_leave

/*
 *==============================================================================
 * global variables
 *==============================================================================
 */

	.section .bss.crsec_nest_cnt, "aw", %nobits

	.type crsec_nest_cnt, %object
crsec_nest_cnt:
	.long 0
	.size crsec_nest_cnt, . - crsec_nest_cnt

/*
 *==============================================================================
 * crsec_enter
 *==============================================================================
 */

	.section .text.crsec_enter, "ax", %progbits
	.type crsec_enter, %function
	.thumb_func
crsec_enter:

	cpsid i
	ldr r1, =crsec_nest_cnt
	ldr r0, [r1]
	adds r0, #1
	str r0, [r1]
	bx lr

	.size crsec_enter, . - crsec_enter

/*
 *==============================================================================
 * crsec_leave
 *==============================================================================
 */

	.section .text.crsec_leave, "ax", %progbits
	.type crsec_leave, %function
	.thumb_func
crsec_leave:

	ldr r1, =crsec_nest_cnt
	ldr r0, [r1]
	subs r0, #1
	str r0, [r1]
	bne crsec_leave_end
	cpsie i
crsec_leave_end:
	bx lr

	.size crsec_leave, . - crsec_leave

/*
 *==============================================================================
 * EOF
 *==============================================================================
 */
	.end
