#include "MKL25Z4.h"
#include "wdog.h"
#include "soc_def.h"
#include "res_alloc.h"
#include "led.h"
#include "sin_lut.h"

#define PIT_EXPT_PRI		2u
#define CMP_EXPT_PRI		2u

#define PIT0_MOD			3000ul

//TODO ???
#define DAC0_DAT0			(*((volatile uint16_t *) ???))

#define CMP_EXPT_CNT_MOD	50u

uint16_t sin_lut_ind, cmp_expt_cnt, disp_cnt;
const uint16_t sin_lut[SIN_LUT_DATA_LEN] = {SIN_LUT_DATA};

void __attribute__ ((interrupt)) PIT_IRQHandler(void)
{
	//TODO: ???

	DAC0_DAT0 = sin_lut[sin_lut_ind];

	sin_lut_ind++;
	if (sin_lut_ind == SIN_LUT_DATA_LEN) {
		sin_lut_ind = 0;
	}
}

void __attribute__ ((interrupt)) CMP0_IRQHandler(void)
{
	//TODO:  povolte preruseni pro sestupnou hranu; vymazte priznak presuseni pro sestupnou hranu;
	//???

	cmp_expt_cnt++;
	if (cmp_expt_cnt == CMP_EXPT_CNT_MOD) {
		cmp_expt_cnt = 0;
		led_bindisp(++disp_cnt);
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();

	//TODO: NVIC setup - preruseni z PITu
	//???
	//???

	//TODO: NVIC setup - preruseni z CMP
	//???
	//???

	//TODO: v registru C0 povolte DAC
	//???

	//PIT setup
	//TODO: zastaveni casovani v debug modu; modulo hodnota; povoleni preruseni; povoleni timeru
	//???
	//???
	//???


	//TODO: nastavte vhodny BNC pro CMP_OUT (napoveda - PCR, MUX, Pinout)
	//???

	//CMP setup
	//TODO: povolte DAC; output voltage 0-64
	//???
	//TODO: plus input MUX - vhodny BNC; minus input MUX - 6-bit DAC0 reference
	//???
	//TODO: povolte preruseni pro sestupnou hranu
	//???
	//TODO: povolte Comparator Output Pin; povolte Comparator Module
	//???

	while (1) {
		wdog_refresh();
	}

	return 0;
}
