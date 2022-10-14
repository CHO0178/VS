/*
 * Name: adc_bm.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "adc_bm.h"

#include "soc_def.h"
#include "crsec.h"
#include "bme.h"

uint16_t adc_bm_read(ADC_BM_MODE adc_mode, uint8_t adc_chan)
{
	uint16_t conv_val;

	crsec_enter();

	switch (adc_mode) {
	case ADC_BM_MODE_8BIT_SE:

		ADC0->CFG1 = ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(ADC_CFG1_MODE_VAL_8BIT) |
					ADC_CFG1_ADICLK(ADC_CFG1_ICLK_VAL_ADACK);
		ADC0->SC1[0u] = ADC_SC1_ADCH(adc_chan);

		break;
	case ADC_BM_MODE_16BIT_SE:

		ADC0->CFG1 = ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(ADC_CFG1_MODE_VAL_16BIT) |
					ADC_CFG1_ADICLK(ADC_CFG1_ICLK_VAL_ADACK);
		ADC0->SC1[0u] = ADC_SC1_ADCH(adc_chan);

		break;
	case ADC_BM_MODE_8BIT_DIFF:

		ADC0->CFG1 = ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(ADC_CFG1_MODE_VAL_8BIT) |
					ADC_CFG1_ADICLK(ADC_CFG1_ICLK_VAL_ADACK);
		ADC0->SC1[0u] = ADC_SC1_DIFF_MASK | ADC_SC1_ADCH(adc_chan);

		break;
	case ADC_BM_MODE_16BIT_DIFF:

		ADC0->CFG1 = ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(ADC_CFG1_MODE_VAL_16BIT) |
					ADC_CFG1_ADICLK(ADC_CFG1_ICLK_VAL_ADACK);
		ADC0->SC1[0u] = ADC_SC1_DIFF_MASK | ADC_SC1_ADCH(adc_chan);

		break;
	default:
		return 0;
	}

	while (!BME_UBFX_W(&ADC0->SC1[0u], ADC_SC1_COCO_SHIFT, 1u));
	conv_val = ADC0->R[0u];

	crsec_leave();

	return conv_val;
}
