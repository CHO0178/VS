/*
 * Name: adc_bm.h
 * Author: Martin Stankus
 *
 */

#ifndef _ADC_BM_H_
#define _ADC_BM_H_

#include <stdint.h>

typedef enum {
	ADC_BM_MODE_8BIT_SE,
	ADC_BM_MODE_16BIT_SE,
	ADC_BM_MODE_8BIT_DIFF,
	ADC_BM_MODE_16BIT_DIFF
} ADC_BM_MODE;

uint16_t adc_bm_read(ADC_BM_MODE adc_mode, uint8_t adc_chan);

#endif /* _ADC_BM_H_ */
