/*
 * Name: vcom_os.c
 * Author: Martin Stankus
 *
 */

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "queue.h"

#include "vcom_os.h"
#include "vcom_os_priv.h"

#include "res_alloc.h"

#include "assert.h"
#include "expt.h"
#include "bme.h"

static SemaphoreHandle_t vcom_os_entry_sem;
static EventGroupHandle_t vcom_os_tx_evt;

static volatile QueueHandle_t vcom_os_rx_que;

static void __attribute__ ((interrupt)) vcom_os_uart_handler(void)
{
	BaseType_t wake_req = pdFALSE;
	uint8_t rx_data;

	UART0->S1 = UART0_S1_OR_MASK | UART0_S1_NF_MASK | UART0_S1_FE_MASK;

	if (BME_UBFX_B(&UART0->S1, UART0_S1_RDRF_SHIFT, 1u)) {
		rx_data = UART0->D;
		xQueueSendFromISR(vcom_os_rx_que, &rx_data, &wake_req);
		portYIELD_FROM_ISR(wake_req);
	}
}

static void __attribute__ ((interrupt)) vcom_os_dma_handler(void)
{
	BaseType_t wake_req = pdFALSE;

	DMA0->DMA[VCOM_DMA_CHAN].DSR_BCR = DMA_DSR_BCR_DONE_MASK;

	assert(xEventGroupSetBitsFromISR(vcom_os_tx_evt, 1u, &wake_req) == pdPASS);
	portYIELD_FROM_ISR(wake_req);
}

void vcom_os_init(QueueHandle_t rx_que, uint8_t expt_pri)
{
	assert(expt_pri < (1u << __NVIC_PRIO_BITS));

	vcom_os_entry_sem = xSemaphoreCreateMutex();
	assert(vcom_os_entry_sem != NULL);

	vcom_os_tx_evt = xEventGroupCreate();
	assert(vcom_os_tx_evt != NULL);

	PORT_UART0_RX->PCR[IOIND_UART0_RX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT2);
	PORT_UART0_TX->PCR[IOIND_UART0_TX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT2);

	UART0->BDH = UART0_BDH_SBR(VCOM_UART_SBR >> 8u);
	UART0->BDL = UART0_BDL_SBR(VCOM_UART_SBR);
	UART0->C5 = UART0_C5_TDMAE_MASK;

	if (rx_que != NULL) {
		vcom_os_rx_que = rx_que;

		UART0->C3 = UART0_C3_ORIE_MASK | UART0_C3_NEIE_MASK | UART0_C3_FEIE_MASK;
		UART0->C2 = UART0_C2_RIE_MASK | UART0_C2_TE_MASK | UART0_C2_RE_MASK;

		expt_vect_set(UART0_IRQn, vcom_os_uart_handler);
		NVIC_SetPriority(UART0_IRQn, expt_pri);
		NVIC_EnableIRQ(UART0_IRQn);
	} else {
		UART0->C2 = UART0_C2_TE_MASK;
	}

	DMAMUX0->CHCFG[VCOM_DMA_CHAN] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_UART0_TX);
	DMA0->DMA[VCOM_DMA_CHAN].DAR = (uint32_t) &UART0->D;

	expt_vect_set(VCOM_DMA_IRQN, vcom_os_dma_handler);
	NVIC_SetPriority(VCOM_DMA_IRQN, expt_pri);
	NVIC_EnableIRQ(VCOM_DMA_IRQN);
}

void vcom_os_send(const void *data, uint16_t len)
{
	assert(xSemaphoreTake(vcom_os_entry_sem, portMAX_DELAY) == pdPASS);

	DMA0->DMA[VCOM_DMA_CHAN].SAR = (uint32_t) data;
	DMA0->DMA[VCOM_DMA_CHAN].DSR_BCR = DMA_DSR_BCR_BCR(len);
	DMA0->DMA[VCOM_DMA_CHAN].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_SINC_MASK | DMA_DCR_SSIZE(1u) | DMA_DCR_DSIZE(1u) | DMA_DCR_D_REQ_MASK;

	xEventGroupWaitBits(vcom_os_tx_evt, 1u, pdTRUE, pdFALSE, portMAX_DELAY);

	assert(xSemaphoreGive(vcom_os_entry_sem) == pdPASS);
}
