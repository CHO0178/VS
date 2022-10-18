#include "MKL25Z4.h"
#include "wdog.h"
#include "soc_def.h"
#include "res_alloc.h"
#include "sin_lut.h"

#define PIT_EXPT_PRI		2u

#define PIT0_MOD			3000ul

//TODO: misto ??? napiste absolutni adresu DAC0_DAT0L registru v hexa
//pretypovani na pointer + dereferencni operator (adresa)
#define DAC0_DAT0			(*((volatile uint16_t *) ???))

uint16_t sin_lut_ind;
const uint16_t sin_lut[SIN_LUT_DATA_LEN] = {SIN_LUT_DATA};

void __attribute__ ((interrupt)) PIT_IRQHandler(void)
{
	//TODO: vymazte priznak preruseni PITu
	//??

	//zapise hodnotu z look up table na adresu registru
	DAC0_DAT0 = sin_lut[sin_lut_ind];

	sin_lut_ind++;
	if (sin_lut_ind == SIN_LUT_DATA_LEN) {
		sin_lut_ind = 0;
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	//TODO: v registru C0 povolte DAC
	//??

	//NVIC setup preruseni z PITu
	//??
	//??

	//PIT setup - casova zakladna
	//nastavte v registrech PITu - zastaveni casovani v debug modu; modulo hodnota; povoleni preruseni; povoleni timeru
	//??
	//??
	//??

	while (1) {
		wdog_refresh();
	}

	return 0;
}
