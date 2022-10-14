/*
 * Name: expt.h
 * Author: Martin Stankus
 *
 */

#ifndef _EXPT_H_
#define _EXPT_H_

#include "MKL25Z4.h"

typedef void (*EXPT_HANDLER)(void);

void Reset_Handler(void);
void Default_Handler(void);

void NMI_Handler(void);
void HardFault_Handler(void);
void SVCall_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA0_IRQHandler(void);
void DMA1_IRQHandler(void);
void DMA2_IRQHandler(void);
void DMA3_IRQHandler(void);
void Reserved20_IRQHandler(void);
void FTFA_IRQHandler(void);
void LVD_LVW_IRQHandler(void);
void LLWU_IRQHandler(void);
void I2C0_IRQHandler(void);
void I2C1_IRQHandler(void);
void SPI0_IRQHandler(void);
void SPI1_IRQHandler(void);
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void ADC0_IRQHandler(void);
void CMP0_IRQHandler(void);
void TPM0_IRQHandler(void);
void TPM1_IRQHandler(void);
void TPM2_IRQHandler(void);
void RTC_IRQHandler(void);
void RTC_Seconds_IRQHandler(void);
void PIT_IRQHandler(void);
void Reserved39_IRQHandler(void);
void USB0_IRQHandler(void);
void DAC0_IRQHandler(void);
void TSI0_IRQHandler(void);
void MCG_IRQHandler(void);
void LPTMR0_IRQHandler(void);
void Reserved45_IRQHandler(void);
void PORTA_IRQHandler(void);
void PORTD_IRQHandler(void);

void expt_vect_set(IRQn_Type irqn, EXPT_HANDLER handler);
uint32_t expt_disable(void);
void expt_restore(uint32_t primask);

#endif /* _EXPT_H_ */
