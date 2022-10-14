#include "MKL25Z4.h"
#include "wdog.h"
#include "soc_def.h"
#include "res_alloc.h"

//kanaly PITu
#define LEDR_PIT_CHAN	0u	//pro red led
#define LEDG_PIT_CHAN	1u	//pro green led

//makra pro POCET TAKTU HODIN
#define LEDR_PERIOD		24000000ul	//pro periodu RED LED 0.5s
#define LEDG_PERIOD		18000000ul	//pro periodu GREEN LED 0.375s

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	
	//TODO: RED LED - periferie PORT (PCR registr - nastavte jako alternativu GPIO)
	//??
	//TODO: RED LED - periferie GPIO (PDDR registr - nastavte vystupni smer)
	//??
	
	//TODO: GREEN LED - periferie PORT (PCR registr - nastavte jako alternativu GPIO)
	//??
	//TODO: GREEN LED - periferie GPIO (PDDR registr - nastavte vystupni smer)
	//??
	
	//TODO: periferie PIT setup - Refman Chapter 32
	//MCR registr - nastavte bit, ktery zastavi casovani, kdyz jde do debug modu
	//??
	//LDVAL registr - channel RED LED  - zapiste hodnotu pro dekrementaci (tzv. modulo hodnota = pocet taktu hodin - 1)
	//??
	//LDVAL registr - channel GREEN LED - zapiste hodnotu pro dekrementaci (tzv. modulo hodnota = pocet taktu hodin - 1)
	//??
	//TCTRL registr - channel RED LED - povoleni timeru
	//??
	//TCTRL registr - channel GREEN LED - povoleni timeru
	//??
	

	while (1) {
		if(PIT->CHANNEL[LEDR_PIT_CHAN].TFLG & PIT_TFLG_TIF_MASK)
		{
			//PIT periferie TFLG registr - vymazte priznak docitani timeru
			//??
			
			//periferie GPIO (PTOR registr) - zapinani/vypinani RED LED
			//??
			
		}
		
		if(PIT->CHANNEL[LEDG_PIT_CHAN].TFLG & PIT_TFLG_TIF_MASK)
		{
			//PIT periferie TFLG registr - vymazte priznak docitani
			//??
						
			//periferie GPIO (PTOR registr) - zapinani/vypinani GREEN LED
			//??	
		}
		
		wdog_refresh();
	}

	return 0;
}