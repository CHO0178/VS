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
	led_init();
	//(0) nastavte zastavení PIT v debug rezimu a povolte periferii PIT
	PIT->MCR = PIT_MCR_FRZ_MASK;
	//(0) nastavte modulo pro obnovovani na 1.5s (LDVAL, busFrequency 24MHz)
	PIT->CHANNEL[0].LDVAL = 36000000ul;
	//(0) povolte casovac PIT a generovani interruptu (TCTRL)
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;
	PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_CHN_MASK;
	//(0) povolte v prijem preruseni v periferii NVIC
	NVIC_EnableIRQ(PIT_IRQn);
	NVIC_SetPriority(PIT_IRQn,2);
	//(1) nastavte druhý kanál na periodu 2s
	PIT->CHANNEL[1].LDVAL = 48000000ul;
	PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;
	PIT->CHANNEL[1].TCTRL = ~PIT_TCTRL_CHN_MASK;
	//(2) nastavte èas systicku na 0.5s
//	SysTick->LOAD = 1500000 - 1u;
//	SysTick->VAL = 0ul;
//	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
	while (1) {
		wdog_refresh();
	}

	return 0;
}


//(0) vytvorte interrupt handler pro casovac PIT
//(0) preklopte hodnotu diodu 0
//(1) podle zdroje preklopte diodu 0 nebo 1 nebo 0 i 1 zaroven

void PIT_IRQHandler()
{
	if(PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK){
		PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK;
		led_toggle(LED_ID2);
	}
	if(PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK){
		PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
		led_toggle(LED_ID1);
	}


}

//(2) vytvoøte handler pro pøerušení ze systicku a pøepínejte ledku 2

void SysTick_Handler()
{
	SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;
	led_toggle(LED_ID3);
}

