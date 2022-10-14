#include "MKL25Z4.h"
#include "wdog.h"
#include "soc_def.h"
#include "res_alloc.h"
#include "led.h"

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();

	//ADC setup
	//CFG1 registr - delicka 4
	//??
	//SC3 registr - povoleni kontinualniho prevodu
	//??
	////SC1[0u] registr - vyber kanalu (potenciometer 1 - pinout SEn)
	//??

	while (1) {
		//co overuje if podminka?
		if (ADC0->SC1[0u] & ADC_SC1_COCO_MASK) {
			//vyzkousejte obe varianty pro 1/0 LED1-8
			//1.varianta
			//k cemu slouzi R registr?
			//vyzkousejte citlivost 0.015f a 0.3f
			led_bgraph(ADC0->R[0u] * 0.035f);
			//2.varianta - led_bgraph() zakomentujte
			//led_bindisp(ADC0->R[0u]);
		}
		wdog_refresh();
	}

	return 0;
}
