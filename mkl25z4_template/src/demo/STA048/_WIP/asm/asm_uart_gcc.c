/*
 * Name: asm_uart_gcc.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"
#include "asm_uart.h"

void uart_init_c(UART_Type *uart, uint16_t sbr)
{
	uart->BDH = UART_BDH_SBR(sbr >> 8);
	uart->BDL = UART_BDL_SBR(sbr);
	uart->C2 = UART_C2_TE_MASK;
}

void uart_send_c(UART_Type *uart, uint8_t data)
{
	while (!(uart->S1 & UART_S1_TDRE_MASK)) {}
	uart->D = data;
}
