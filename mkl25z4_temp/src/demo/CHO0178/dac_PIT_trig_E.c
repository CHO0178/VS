#include "wdog.h"
#include "littleHelper.h"
#include "MKL25Z4.h"
#include "sin_lut.h"
/*
DAC
	DAT0L
		DATA0			lower data pro pøevod
	DAT0H
		DATA1			higher data pro pøevod
	C0
		DACEN			povolí použití 12bit DAC


*/
#define PIT_EXPT_PRI		2u
#define PIT0_MOD			3000ul


uint16_t sin_lut_ind;
const uint16_t sin_lut[SIN_LUT_DATA_LEN] = {SIN_LUT_DATA};

void setupNVICandPIT();

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	// povolte použití 12bit DAC

	setupNVICandPIT();


	while (1) {
		wdog_refresh();
	}

	return 0;
}


void setupNVICandPIT()
{
	NVIC_SetPriority(PIT_IRQn, PIT_EXPT_PRI);
	NVIC_EnableIRQ(PIT_IRQn);

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0u].LDVAL = PIT0_MOD - 1u;
	PIT->CHANNEL[0u].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
}

void __attribute__ ((interrupt)) PIT_IRQHandler(void)
{
	PIT->CHANNEL[0u].TFLG = PIT_TFLG_TIF_MASK;

	// vložte hodnoty z sin_lut do registrù DAT0L, DAT0H


	// inkrementujte index ukazující na pøíští hodnotu použitou po DAC

	// pøi pøesáhnutí indexu hodnoty SIN_LUT_DATA_LEN vynuluj index

}
