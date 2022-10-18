#include "MKL25Z4.h"
#include "wdog.h"
#include "soc_def.h"
#include "res_alloc.h"
#include "sin_lut.h"

#define DAC0_EXPT_PRI		2u

#define PIT0_MOD			3000ul
//TODO: misto ??? napiste absolutni adresu DAC0_DAT0L registru v hexa
#define DAC0_DAT0			(*((volatile uint16_t *) ???))

//TODO: misto ??? napiste absolutni adresu DAC0_DAT1L registru v hexa
#define DAC0_DAT1			(*((volatile uint16_t *) ???))

uint16_t sin_lut_ind;
const uint16_t sin_lut[SIN_LUT_DATA_LEN] = {SIN_LUT_DATA};

void __attribute__ ((interrupt)) DAC0_IRQHandler(void)
{
	//co overuje if podminka?
	if (DAC0->SR & DAC_SR_DACBFRPTF_MASK) {
		//TODO: vymazte priznak DAC Buffer Read Pointer Top Position Flag
		//??

		DAC0_DAT1 = sin_lut[sin_lut_ind];
	}

	//co overuje if podminka?
	if (DAC0->SR & DAC_SR_DACBFRPBF_MASK) {
		//TODO: vymazte priznak DAC Buffer Read Pointer Bottom Position Flag
		//??

		DAC0_DAT0 = sin_lut[sin_lut_ind];
	}

	sin_lut_ind++;
	if (sin_lut_ind == SIN_LUT_DATA_LEN) {
		sin_lut_ind = 0;
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	//NVIC setup - preruseni z DAC0
	//??
	//??

	//DAC0 registry - povolte buffer read pointer; povolte DAC; povolte Top Flag a Bottom Flag preruseni
	//??
	//??

	//PIT setup
	//zastaveni casovani v debug modu; modulo hodnota; povoleni timeru
	//??
	//??
	//??

	while (1) {
		wdog_refresh();
	}

	return 0;
}
