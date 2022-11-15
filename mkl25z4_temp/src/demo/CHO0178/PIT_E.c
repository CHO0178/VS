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

	//(0) nastavte zastaven� v debug rezimu

	//(0) nastavte modulo pro obnovovani na 1.5s (LDVAL, busFrequency 24MHz)

	//(0) povolte casovac PIT a generovani interruptu (TCTRL)

	//(1) nastavte druh� kan�l na periodu 2s


	//(2) nastavte �as systicku na 0.5s

	while (1) {

		wdog_refresh();
	}

	return 0;
}


//(0) vytvorte interrupt handler pro casovac PIT
//(0) preklopte hodnotu diodu 0
//(1) podle zdroje preklopte diodu 0 nebo 1 nebo 0 i 1 zaroven


//(2) vytvo�te handler pro p�eru�en� ze systicku a p�ep�nejte ledku 2

