/*
 * Name: adc_os.h
 * Author: Martin Stankus
 *
 */

#ifndef _ADC_OS_H_
#define _ADC_OS_H_

#include <stdint.h>

typedef enum {
	ADC_OS_MODE_8BIT_SE,
	ADC_OS_MODE_16BIT_SE,
	ADC_OS_MODE_8BIT_DIFF,
	ADC_OS_MODE_16BIT_DIFF
} ADC_OS_MODE;

void adc_os_init(uint8_t expt_pri);
uint16_t adc_os_read(ADC_OS_MODE adc_mode, uint8_t adc_chan);

#endif /* _ADC_OS_H_ */
