/*
 * Name: tsi_os.c
 * Author: Martin Stankus
 *
 */

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "tsi_os.h"
#include "tsi_os_priv.h"

#include "assert.h"
#include "expt.h"

static volatile uint16_t tsi_os_conv_val;

static SemaphoreHandle_t tsi_os_entry_sem;
static EventGroupHandle_t tsi_os_conv_evt;

static void __attribute__ ((interrupt)) tsi_os_scan_handler(void)
{
	BaseType_t wake_req = pdFALSE;

	tsi_os_conv_val = TSI0->DATA;
	TSI0->GENCS = TSI_GENCS_COMPVAL | TSI_GENCS_EOSF_MASK;

	assert(xEventGroupSetBitsFromISR(tsi_os_conv_evt, 1u, &wake_req) == pdPASS);
	portYIELD_FROM_ISR(wake_req);
}

void tsi_os_init(uint8_t expt_pri)
{
	assert(expt_pri < (1u << __NVIC_PRIO_BITS));

	tsi_os_entry_sem = xSemaphoreCreateMutex();
	assert(tsi_os_entry_sem != NULL);

	tsi_os_conv_evt = xEventGroupCreate();
	assert(tsi_os_conv_evt != NULL);

	TSI0->GENCS = TSI_GENCS_COMPVAL;

	expt_vect_set(TSI0_IRQn, tsi_os_scan_handler);
	NVIC_SetPriority(TSI0_IRQn, expt_pri);
	NVIC_EnableIRQ(TSI0_IRQn);
}

uint8_t tsi_os_read(uint16_t thr)
{
	uint8_t res = 0u;

	assert(xSemaphoreTake(tsi_os_entry_sem, portMAX_DELAY) == pdPASS);

	TSI0->DATA = TSI_DATA_TSICH(TSI_CHAN_LEFT) | TSI_DATA_SWTS_MASK;

	xEventGroupWaitBits(tsi_os_conv_evt, 1u, pdTRUE, pdFALSE, portMAX_DELAY);

	if (tsi_os_conv_val >= thr) {
		res |= TSI_OS_MASK_LEFT;
	}

	TSI0->DATA = TSI_DATA_TSICH(TSI_CHAN_RIGHT) | TSI_DATA_SWTS_MASK;

	xEventGroupWaitBits(tsi_os_conv_evt, 1u, pdTRUE, pdFALSE, portMAX_DELAY);

	if (tsi_os_conv_val >= thr) {
		res |= TSI_OS_MASK_RIGHT;
	}

	assert(xSemaphoreGive(tsi_os_entry_sem) == pdPASS);

	return res;
}
