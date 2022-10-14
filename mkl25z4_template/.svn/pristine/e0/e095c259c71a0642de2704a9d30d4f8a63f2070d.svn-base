/*
 * Name: _asm_testbench.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"
#include "wdog.h"
#include "port.h"
#include "io_dock.h"
#include "uart.h"
#include "str.h"

#include "asm_fib.h"
#include "asm_glob.h"
#include "asm_max.h"
#include "asm_sum.h"
#include "asm_uart.h"

#define ARRAY_LEN		5
#define ARRAY_DATA		{5, 0, 7, 9, 3}

#define FIB_LIMIT		10

int main(void)
{
	uint64_t sum_res_c, sum_res_asm;
	uint32_t msg_ind, msg_len, glob_val, fib_res_c, fib_res_asm;

	uint16_t array[ARRAY_LEN] = ARRAY_DATA;
	uint16_t array_max_c, array_max_asm;

	char *msg_c = "hello world, c impl\n";
	char *msg_asm = "hello world, asm impl\n";

	wdog_set(WDOG_CONF_LPOCLK_1024_CYCLES);

	PORT_UART1_TX->PCR[IOIND_UART1_TX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	uart_init_asm(UART1, UART_SBR_CLK24M_115200BD);

	msg_len = strlen(msg_c);
	for (msg_ind = 0; msg_ind < msg_len; msg_ind++) {
		uart_send_c(UART1, msg_c[msg_ind]);
	}

	msg_len = strlen(msg_asm);
	for (msg_ind = 0; msg_ind < msg_len; msg_ind++) {
		uart_send_asm(UART1, msg_asm[msg_ind]);
	}

	sum_res_c = sum_c(0xFFFFFFFFul, 0xFFFFFFFFul);
	sum_res_asm = sum_asm(0xFFFFFFFFul, 0xFFFFFFFFul);

	array_max_c = max_c(array, ARRAY_LEN);
	array_max_asm = max_asm(array, ARRAY_LEN);

	glob_val = glob_get_asm();
	glob_set_c(0xC0C0C0C0ul);
	glob_val = glob_get_asm();
	glob_set_asm(0xDFDFDFDFul);
	glob_val = glob_get_c();

	fib_res_c = fib_c(FIB_LIMIT);
	fib_res_asm = fib_asm(FIB_LIMIT);

	while (1) {
		wdog_refresh();
	}

	return 0;
}
