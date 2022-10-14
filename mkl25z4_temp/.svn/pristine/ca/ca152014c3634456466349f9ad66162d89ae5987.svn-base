/*
 * Name: rtos_que_sem.c
 * Author: Martin Stankus
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "wdog.h"
#include "soc_def.h"
#include "res_alloc.h"
#include "adc_os.h"
#include "print.h"
#include "str.h"
#include "term.h"

#define TASK_POT1_PRI			(configMAX_PRIORITIES - 1)
#define TASK_POT2_PRI			(configMAX_PRIORITIES - 1)
#define TASK_PRN_PRI			1
#define TASK_WDOG_PRI			0

#define TASK_POT1_MEAS_PERIOD	40u
#define TASK_POT2_MEAS_PERIOD	10u

#define TASK_POT1_MEAS_Q_LEN	64u
#define TASK_POT2_MEAS_Q_LEN	64u

#define MSG_LEN_LIM				64u

typedef struct {
	QueueHandle_t meas_q;
	TickType_t meas_period;
	uint8_t adc_chan;
} TASK_POT_CONF;

typedef struct {
	uint8_t id;
	uint8_t val;
} MEAS;

void task_pot(void *prm) {
	MEAS meas;
	TASK_POT_CONF *conf = prm;

	meas.id = conf->adc_chan;

	while (1) {
		meas.val = adc_os_read(ADC_OS_MODE_8BIT_SE, conf->adc_chan);
		xQueueSend(conf->meas_q, &meas, 0);
		vTaskDelay(conf->meas_period);
	}
}

void task_prn(void *prm) {
	QueueHandle_t meas_q;
	QueueSetHandle_t *qset = prm;
	MEAS meas;

	char msg[MSG_LEN_LIM];
	uint8_t msg_ind, msg_len;

	PORT_UART1_TX->PCR[IOIND_UART1_TX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	UART1->BDH = UART_BDH_SBR(UART_SBR_115200BD_CLK24M >> 8u);
	UART1->BDL = UART_BDL_SBR(UART_SBR_115200BD_CLK24M);
	UART1->C2 = UART_C2_TE_MASK;

	while (1) {
		meas_q = xQueueSelectFromSet(*qset, portMAX_DELAY);
		assert(xQueueReceive(meas_q, &meas, 0) == pdPASS);

		msg_ind = 0u;
		msg_len = 0u;

		msg_len += strcpy(&msg[msg_len], TERM_CU_OFF);

		switch (meas.id) {
		case ADC_CHAN_POT1:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(1,1));
			msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_ULINE_ON));
			msg_len += strcpy(&msg[msg_len], "pot1:");
			break;
		case ADC_CHAN_POT2:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(2,1));
			msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_ULINE_ON));
			msg_len += strcpy(&msg[msg_len], "pot2:");
			break;
		default:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(3,1));
			msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_ULINE_ON));
			msg_len += strcpy(&msg[msg_len], "unknown:");
			break;
		}

		msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_ULINE_OFF));

		msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_FGC_RED));
		msg_len += print_udec(&msg[msg_len], meas.val);
		msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_FGC_WHITE));

		msg_len += strcpy(&msg[msg_len], TERM_EL(TERM_EL_ARG_CUR_TO_END));

		while (msg_ind < msg_len) {
			if (UART1->S1 & UART_S1_TDRE_MASK) {
				UART1->D = msg[msg_ind++];
			}
		}
	}
}

void task_wdog(void *prm) {
	while (1) {
		wdog_refresh();
		taskYIELD();
	}
}

int main(void)
{
	TASK_POT_CONF task_pot1_conf, task_pot2_conf;
	QueueSetHandle_t task_prn_qset;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	task_pot1_conf.meas_q = xQueueCreate(TASK_POT1_MEAS_Q_LEN, sizeof(MEAS));
	task_pot1_conf.meas_period = TASK_POT1_MEAS_PERIOD;
	task_pot1_conf.adc_chan = ADC_CHAN_POT1;

	task_pot2_conf.meas_q = xQueueCreate(TASK_POT2_MEAS_Q_LEN, sizeof(MEAS));
	task_pot2_conf.meas_period = TASK_POT2_MEAS_PERIOD;
	task_pot2_conf.adc_chan = ADC_CHAN_POT2;

	vQueueAddToRegistry(task_pot1_conf.meas_q, "pot1_meas_q");
	vQueueAddToRegistry(task_pot2_conf.meas_q, "pot2_meas_q");

	task_prn_qset = xQueueCreateSet(TASK_POT1_MEAS_Q_LEN + TASK_POT2_MEAS_Q_LEN);
	xQueueAddToSet(task_pot1_conf.meas_q, task_prn_qset);
	xQueueAddToSet(task_pot2_conf.meas_q, task_prn_qset);

	adc_os_init(0u);

	assert(xTaskCreate(task_pot, "task_pot1", configMINIMAL_STACK_SIZE, &task_pot1_conf, TASK_POT1_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_pot, "task_pot2", configMINIMAL_STACK_SIZE, &task_pot2_conf, TASK_POT2_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_prn, "task_prn", configMINIMAL_STACK_SIZE, &task_prn_qset, TASK_PRN_PRI, NULL) == pdPASS);

	assert(xTaskCreate(task_wdog, "task_wdog", configMINIMAL_STACK_SIZE, NULL, TASK_WDOG_PRI, NULL) == pdPASS);

	vTaskStartScheduler();

	return 0;
}
