/*
 * Name: rtos_fmeas_wgen.c
 * Author: Martin Stankus
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "wdog.h"
#include "io_dock.h"
#include "port_defs.h"
#include "uart_defs.h"
#include "adc.h"
#include "print.h"
#include "str.h"
#include "term.h"

#define TASK_FMEAS_PRI			(configMAX_PRIORITIES - 1)
#define TASK_WGEN_PRI			(configMAX_PRIORITIES - 1)
#define TASK_TX_PRI				1
#define TASK_RX_PRI				0
#define TASK_WDOG_PRI			0

#define RX_Q_LEN				8
#define TX_Q_LEN				32

#define MSG_LEN_LIM				64

typedef struct {
	SemaphoreHandle_t uart_sem;
	QueueHandle_t rx_q;
	QueueHandle_t tx_q;
} APP_CONF;

typedef struct {
	uint16_t freq;
	uint16_t time;
} RX_PKT;

typedef struct {
	enum {
		TX_PKT_TYPE_FMEAS,
		TX_PKT_TYPE_WGEN
	} type;
	union {
		struct {
			uint32_t period;
		} fmeas;
		struct {
			uint16_t freq;
			uint16_t time;
		} wgen;
	} data;
} TX_PKT;

uint32_t time_total;

void __attribute__ ((interrupt)) TPM1_IRQHandler(void)
{
	static uint32_t time, time_chan;

	if (TPM1->STATUS & TPM_STATUS_TOF_MASK) {
		TPM1->STATUS = TPM_STATUS_TOF_MASK;
		time += 65536ul;
	} else {
		TPM1->STATUS = TPM_STATUS_CH0F_MASK;
		time_total = time + TPM1->CONTROLS[0].CnV - time_chan;
		time_chan = TPM1->CONTROLS[0].CnV;
		time = 0;
	}
}

void tmrcb_fmeas(TimerHandle_t tmr) {
	TX_PKT tx_pkt;
	APP_CONF *app_conf = pvTimerGetTimerID(tmr);

	tx_pkt.type = TX_PKT_TYPE_FMEAS;
	tx_pkt.data.fmeas.period = time_total;
	xQueueSend(app_conf->tx_q, &tx_pkt, 0);
}

void task_wgen(void *prm) {
	RX_PKT rx_pkt;
	TX_PKT tx_pkt;
	APP_CONF *conf = prm;
	uint8_t delay;

	tx_pkt.type = TX_PKT_TYPE_FMEAS;

	while (1) {
		tx_pkt.payload = 0;
		xQueueSend(conf->tx_q, &tx_pkt, 0);

		xQueueReceive(conf->rx_q, &delay, portMAX_DELAY);

		tx_pkt.payload = delay;
		xQueueSend(conf->tx_q, &tx_pkt, 0);

		//enable tpm

		vTaskDelay(WGEN_DELAY);

		//disable tpm
	}
}

void task_rx(void *prm) {

}

void task_tx(void *prm) {
	TX_PKT tx_pkt;
	APP_CONF *conf = prm;

	char msg[MSG_LEN_LIM];
	uint8_t msg_ind, msg_len;

	while (1) {
		xQueueReceive(conf->tx_q, &tx_pkt, portMAX_DELAY);

		msg_ind = 0;
		msg_len = 0;

		msg_len += strcpy(&msg[msg_len], TERM_CU_OFF);

		switch (tx_pkt.id) {
		case TX_PKT_ID_FMEASVAL:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(1,1));
			msg_len += strcpy(&msg[msg_len], "inp period: ");
			break;
		case TX_PKT_ID_WGENSTAT:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(2,1));
			msg_len += strcpy(&msg[msg_len], "out freq: ");
			break;
		default:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(3,1));
			msg_len += strcpy(&msg[msg_len], "unknown: ");
			break;
		}

		msg_len += print_udec(&msg[msg_len], tx_pkt.payload);
		msg_len += strcpy(&msg[msg_len], TERM_EL(TERM_EL_ARG_CUR_TO_END));

		xSemaphoreTake(conf->uart_sem, portMAX_DELAY);
		while (msg_ind < msg_len) {
			if (UART1->S1 & UART_S1_TDRE_MASK) {
				UART1->D = msg[msg_ind++];
			}
		}
		xSemaphoreGive(conf->uart_sem);
	}
}

void uart_set(void) {
	PORT_UART0_RX->PCR[IOIND_UART0_RX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT2);
	PORT_UART0_TX->PCR[IOIND_UART0_RX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT2);

	UART0->BDH = UART_BDH_SBR(UART_SBR_115200BD_CLK48M >> 8);
	UART0->BDL = UART_BDL_SBR(UART_SBR_115200BD_CLK48M);
	UART0->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;
}

void ic_set()
{

}

void oc_set(void)
{

}

void task_wdog(void *prm) {
	while (1) {
		wdog_refresh();
		taskYIELD();
	}
}

int main(void)
{
	APP_CONF app_conf;

	wdog_set(WDOG_CONF_LPOCLK_1024_CYCLES);

	uart_set();
	ic_set();
	oc_set();

	app_conf.uart_sem = xSemaphoreCreateMutex();
	app_conf.rx_q = xQueueCreate(RX_Q_LEN, sizeof(uint8_t));
	app_conf.tx_q = xQueueCreate(TX_Q_LEN, sizeof(TX_PKT));

	xTaskCreate(task_fmeas, "task_fmeas", configMINIMAL_STACK_SIZE, &app_conf, TASK_FMEAS_PRI, NULL);
	xTaskCreate(task_wgen, "task_wgen", configMINIMAL_STACK_SIZE, &app_conf, TASK_WGEN_PRI, NULL);
	xTaskCreate(task_rx, "task_rx", configMINIMAL_STACK_SIZE, &app_conf, TASK_RX_PRI, NULL);
	xTaskCreate(task_tx, "task_tx", configMINIMAL_STACK_SIZE, &app_conf, TASK_TX_PRI, NULL);
	xTaskCreate(task_wdog, "task_wdog", configMINIMAL_STACK_SIZE, NULL, TASK_WDOG_PRI, NULL);

	vTaskStartScheduler();

	return 0;
}
