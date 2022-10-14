/*
 * Name: fib_tx.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "wdog.h"

#include "fib_ref.h"
#include "fib.h"
#include "fib_locvar.h"

int main(void)
{
	unsigned int res1, res2, res3;

	wdog_init(WDOG_CONF_DIS);

	fib_ref(&res1, 12);
	fib(&res2, 12);
	fib_locvar(&res3, 12);

	__BKPT();

	return 0;
}
