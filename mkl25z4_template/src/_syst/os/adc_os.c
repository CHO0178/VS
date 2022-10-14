/*
 * Name: adc_os.c
 * Author: Martin Stankus
 *
 */

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"

#include "adc_os.h"
#include "adc_os_priv.h"

#include "soc_def.h"
#include "assert.h"
#include "expt.h"

static volatile uint16_t adc_os_conv_val;

static SemaphoreHandle_t adc_os_entry_sem;
static EventGroupHandle_t adc_os_conv_evt;

static void __attribute__ ((interrupt)) adc_os_conv_handler(void)
{
	BaseType_t wake_req = pdFALSE;

	adc_os_conv_val = ADC0->R[0u];

	assert(xEventGroupSetBitsFromISR(adc_os_conv_evt, 1u, &wake_req) == pdPASS);
	portYIELD_FROM_ISR(wake_req);
}

void adc_os_init(uint8_t expt_pri)
{
	assert(expt_pri < (1u << __NVIC_PRIO_BITS));

	adc_os_entry_sem = xSemaphoreCreateMutex();
	assert(adc_os_entry_sem != NULL);

	adc_os_conv_evt = xEventGroupCreate();
	assert(adc_os_conv_evt != NULL);

	expt_vect_set(ADC0_IRQn, adc_os_conv_handler);
	NVIC_SetPriority(ADC0_IRQn, expt_pri);
	NVIC_EnableIRQ(ADC0_IRQn);
}

uint16_t adc_os_read(ADC_OS_MODE adc_mode, uint8_t adc_chan)
{
	uint16_t res;

	assert(xSemaphoreTake(adc_os_entry_sem, portMAX_DELAY) == pdPASS);

	switch (adc_mode) {
	case ADC_OS_MODE_8BIT_SE:

		ADC0->CFG1 = ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(ADC_CFG1_MODE_VAL_8BIT) |
					ADC_CFG1_ADICLK(ADC_CFG1_ICLK_VAL_ADACK);
		ADC0->SC1[0u] = ADC_SC1_AIEN_MASK | ADC_SC1_ADCH(adc_chan);

		break;
	case ADC_OS_MODE_16BIT_SE:

		ADC0->CFG1 = ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(ADC_CFG1_MODE_VAL_16BIT) |
					ADC_CFG1_ADICLK(ADC_CFG1_ICLK_VAL_ADACK);
		ADC0->SC1[0u] = ADC_SC1_AIEN_MASK |ADC_SC1_ADCH(adc_chan);

		break;
	case ADC_OS_MODE_8BIT_DIFF:

		ADC0->CFG1 = ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(ADC_CFG1_MODE_VAL_8BIT) |
					ADC_CFG1_ADICLK(ADC_CFG1_ICLK_VAL_ADACK);
		ADC0->SC1[0u] = ADC_SC1_AIEN_MASK | ADC_SC1_DIFF_MASK | ADC_SC1_ADCH(adc_chan);

		break;
	case ADC_OS_MODE_16BIT_DIFF:

		ADC0->CFG1 = ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(ADC_CFG1_MODE_VAL_16BIT) |
					ADC_CFG1_ADICLK(ADC_CFG1_ICLK_VAL_ADACK);
		ADC0->SC1[0u] = ADC_SC1_AIEN_MASK | ADC_SC1_DIFF_MASK | ADC_SC1_ADCH(adc_chan);

		break;
	default:

		adc_os_conv_val = 0u;
		xEventGroupSetBits(adc_os_conv_evt, 1u);

		break;
	}

	xEventGroupWaitBits(adc_os_conv_evt, 1u, pdTRUE, pdFALSE, portMAX_DELAY);

	res = adc_os_conv_val;

	assert(xSemaphoreGive(adc_os_entry_sem) == pdPASS);

	return res;
}
