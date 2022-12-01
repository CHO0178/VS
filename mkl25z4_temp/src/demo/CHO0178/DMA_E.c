#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "led.h"

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();
	// DAC - chapter 30
	// povolte periferii DAC (C0)(DACEN = 1)


	// povolte pending IRQ z DMA v NVICu na prioritu 2

	// ADC - chapter 28
	// v analog to digital prevodniku nastavte divideRatio (ADIV = 2) na inputClock/4 a 12-bitovou konverzi(mode = 1)//(CFG1)

	// nastavte v ADC zvolte conversion triger na Hardware trigger(ADTRG = 1) a povolte generování DMA requestu (DMAEN = 1)//(SC2)

	// povolte generovani interuptu(AIEN = 1) a zvoleni vstupu AD4 (ADCH = 4)//(SC1)


	// DMA - chapter 23
	// nastavte zdrojovou adresu na registr R periferie ADC

	// nastavte destination adress register na adresu registru DAC0_DAT0L

	// nastavte pocet opakovani pred vyvoanim preruseni na 500 (BCR = 500)

	// povolte generovani interuptu pri dokonceni prenosu(EINT = 1), peirferii vyzadat prenos(ERQ = 1), nastavte mod prenosu na cycle steal(CS = 1) a velikosti prenasenych hodnot na 2B (SSIZE = 2;DSIZE = 2)

	// DMAMUX chapter 22
	// povolte kanal a nastavte zdroj ADC0 (str. 65)(ENBL)//(CHCFG)


	// SIM chapter 12
	// povolte alternativni trigrovani modulu ADC0(ADC0ALTTRGEN = 1) a nastavte zdroj trigrovani na PIT0 (SOPT7)


	// PIT chapter 32
	// povolte zastaveni casovace behem debugingu

	// nastavte obnovovaci frekvenci na 100us MODULO -> LDVAL

	// povolte timer (TEN = 1)//(TCTRL)


	while (1) {
		wdog_refresh();
	}

	return 0;
}

// vytvorte handler z preruseni periferie DMA ve kterem nastavite hodnotu BCR na 500

void DMA0_IRQHandler(void)
{

}
