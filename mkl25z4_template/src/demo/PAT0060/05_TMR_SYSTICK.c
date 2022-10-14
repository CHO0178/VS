#include "MKL25Z4.h"
#include "wdog.h"
#include "led.h"

#define DISP_PERIOD		1500000ul

int main(void)
{
	uint8_t led_val = 0u;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();

	//LOAD registr - hodnota pro dekrementaci
	//??
	//VAL registr - nastavte na 0
	//??
	//CTRL registr - povolte timer
	//??

	while (1) {
		//napiste if podminku - zda timer docital do konce (zda je zapsana 1 na bitu priznak docitani)
		{
			led_bindisp(++led_val);
		}

		wdog_refresh();
	}

	return 0;
}
