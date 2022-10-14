/*
 * Name: rtos_fmeas_wgen_lite.c
 * Author: Martin Stankus
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "wdog.h"
#include "print.h"
#include "str.h"
#include "term.h"

#define TX_PKT_ID_FMEASVAL		0
#define TX_PKT_ID_WGENSTAT		1

volatile uint32_t time_total;

typedef struct {
	uint32_t id;
	uint32_t payload;
} TX_PKT;

typedef struct {
	SemaphoreHandle_t uart_sem;
	QueueHandle_t rx_q;
	QueueHandle_t tx_q;
} APP_CONF;

void __attribute__ ((interrupt)) TPM0_IRQHandler(void)
{
	static uint32_t time, time_chan;

	if (TPM0->STATUS & TPM_STATUS_TOF_MASK) {
		TPM0->STATUS = TPM_STATUS_TOF_MASK;
		time += 65536ul;
	} else {
		TPM0->STATUS = 1ul << 2;
		time_total = time + TPM0->CONTROLS[2].CnV - time_chan;
		time_chan = TPM0->CONTROLS[2].CnV;
		time = 0;
	}
}

void task_fmeas(void *prm)
{
	TX_PKT tx_pkt;
	APP_CONF *conf = prm;
	tx_pkt.id = TX_PKT_ID_FMEASVAL;

	NVIC_SetPriority(TPM0_IRQn, 0);
	NVIC_EnableIRQ(TPM0_IRQn);

	PORTE->PCR[29] = PORT_PCR_MUX(3);

	TPM0->CONTROLS[2].CnSC = TPM_CnSC_CHIE_MASK | TPM_CnSC_ELSB_MASK;
	TPM0->SC = TPM_SC_TOIE_MASK | TPM_SC_CMOD(1) | TPM_SC_PS(4);

	while (1) {
		tx_pkt.payload = time_total;
		xQueueSend(conf->tx_q, &tx_pkt, 0);
		vTaskDelay(40);
	}
}

void task_wgen(void *prm)
{
	uint32_t rx_data;
	TX_PKT tx_pkt;
	APP_CONF *conf = prm;
	tx_pkt.id = TX_PKT_ID_WGENSTAT;

	PORTE->PCR[23] = PORT_PCR_MUX(3);

	TPM2->CONTROLS[1].CnSC = TPM_CnSC_MSA_MASK | TPM_CnSC_ELSA_MASK;

	while (1) {
		tx_pkt.payload = 0;
		xQueueSend(conf->tx_q, &tx_pkt, 0);

		xQueueReceive(conf->rx_q, &rx_data, portMAX_DELAY);

		tx_pkt.payload = rx_data * 100;
		xQueueSend(conf->tx_q, &tx_pkt, 0);

		TPM2->CNT = 0;
		TPM2->CONTROLS[1].CnV = 0;
		TPM2->MOD = (15000ul / rx_data) - 1;
		TPM2->SC = TPM_SC_CMOD(1) | TPM_SC_PS(4);

		vTaskDelay(400);

		TPM2->SC = 0;
	}
}


void task_rx(void *prm)
{
	uint32_t rx_data;
	APP_CONF *conf = prm;

	while (1) {
		xSemaphoreTake(conf->uart_sem, portMAX_DELAY);
		if (UART0->S1 & UART_S1_RDRF_MASK) {
			rx_data = UART0->D;
			xSemaphoreGive(conf->uart_sem);

			if ((rx_data >= '1') && (rx_data <= '9')) {
				rx_data -= '0';
				xQueueSend(conf->rx_q, &rx_data, 0);
			}

			continue;
		}
		xSemaphoreGive(conf->uart_sem);
	}
}

void task_tx(void *prm)
{
	TX_PKT tx_pkt;
	APP_CONF *conf = prm;

	char msg[64];
	uint8_t msg_ind, msg_len;

	while (1) {
		xQueueReceive(conf->tx_q, &tx_pkt, portMAX_DELAY);

		msg_ind = 0;
		msg_len = 0;

		msg_len += strcpy(&msg[msg_len], TERM_CU_OFF);

		switch (tx_pkt.id) {
		case TX_PKT_ID_FMEASVAL:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(1,1));
			msg_len += strcpy(&msg[msg_len], "fmeas: ");
			break;
		case TX_PKT_ID_WGENSTAT:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(2,1));
			msg_len += strcpy(&msg[msg_len], "wgen: ");
			break;
		default:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(3,1));
			msg_len += strcpy(&msg[msg_len], "unknown: ");
			break;
		}

		msg_len += print_udec(&msg[msg_len], tx_pkt.payload);
		msg_len += strcpy(&msg[msg_len], TERM_EL(TERM_EL_ARG_CUR_TO_END));

		while (msg_ind < msg_len) {
			xSemaphoreTake(conf->uart_sem, portMAX_DELAY);
			if (UART0->S1 & UART_S1_TDRE_MASK) {
				UART0->D = msg[msg_ind++];
			}
			xSemaphoreGive(conf->uart_sem);
		}
	}
}

void task_wdog(void *prm)
{
	while (1) {
		wdog_refresh();
		taskYIELD();
	}
}

int main(void)
{
	APP_CONF app_conf;

	wdog_set(WDOG_CONF_LPOCLK_1024_CYCLES);

	PORTA->PCR[1] = PORT_PCR_MUX(2);
	PORTA->PCR[2] = PORT_PCR_MUX(2);

	UART0->BDH = UART_BDH_SBR(0);
	UART0->BDL = UART_BDL_SBR(26);
	UART0->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;

	app_conf.uart_sem = xSemaphoreCreateMutex();
	app_conf.rx_q = xQueueCreate(8, sizeof(uint32_t));
	app_conf.tx_q = xQueueCreate(8, sizeof(TX_PKT));

	xTaskCreate(task_fmeas, "task_fmeas", 64, &app_conf, 2, NULL);
	xTaskCreate(task_wgen, "task_wgen", 64, &app_conf, 2, NULL);
	xTaskCreate(task_rx, "task_rx", 64, &app_conf, 0, NULL);
	xTaskCreate(task_tx, "task_tx", 64, &app_conf, 1, NULL);
	xTaskCreate(task_wdog, "task_wdog", 64, NULL, 0, NULL);

	vTaskStartScheduler();

	return 0;
}
