/*

PIT
	MCR
		MDIS
		FRZ				freeze in debug (1-allow freeze)
	LTMR64H
	LTMR64H
	LDVAL				load value
	TCTRL
	TFLG

*/

//include libraries
#include "MKL25Z4.h"
#include "led.h"
#include "wdog.h"

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	//(0) inicializujte diody

	//(0) nastavte zastavení v debug rezimu

	//(0) nastavte modulo pro obnovovani na 1.5s (LDVAL, busFrequency 24MHz)

	//(0) povolte casovac PIT a generovani interruptu (TCTRL)

	//(1) nastavte druhý kanál na periodu 2s


	//(2) nastavte èas systicku na 0.5s

	while (1) {

		wdog_refresh();
	}

	return 0;
}


//(0) vytvorte interrupt handler pro casovac PIT
//(0) preklopte hodnotu diodu 0
//(1) podle zdroje preklopte diodu 0 nebo 1 nebo 0 i 1 zaroven


//(2) vytvoøte handler pro pøerušení ze systicku a pøepínejte ledku 2

