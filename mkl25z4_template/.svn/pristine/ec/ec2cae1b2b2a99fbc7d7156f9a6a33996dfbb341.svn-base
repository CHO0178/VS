/*
 * Name: rtos_que_sem_alt.c
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
#include "tsi_os.h"
#include "vcom_os.h"
#include "print.h"
#include "str.h"
#include "term.h"

#define TASK_BGAP_PRI			(configMAX_PRIORITIES - 1)
#define TASK_TEMP_PRI			(configMAX_PRIORITIES - 1)
#define TASK_TOUCH_PRI			(configMAX_PRIORITIES - 1)
#define TASK_PRN_PRI			1
#define TASK_WDOG_PRI			0

#define TASK_BGAP_MEAS_PERIOD	80u
#define TASK_TEMP_MEAS_PERIOD	50u
#define TASK_TOUCH_MEAS_PERIOD	10u

#define TASK_BGAP_MEAS_ID		0u
#define TASK_TEMP_MEAS_ID		1u
#define TASK_TOUCH_MEAS_ID		2u

#define TASK_BGAP_MEAS_Q_LEN	16u
#define TASK_TEMP_MEAS_Q_LEN	16u
#define TASK_TOUCH_MEAS_Q_LEN	16u

#define MSG_LEN_LIM				64u

typedef struct {
	QueueHandle_t meas_q;
	TickType_t meas_period;
	uint8_t adc_chan;
	uint8_t id;
} TASK_ADC_CONF;

typedef struct {
	QueueHandle_t meas_q;
	TickType_t meas_period;
	uint8_t id;
} TASK_TSI_CONF;

typedef struct {
	uint8_t id;
	uint16_t val;
} MEAS;

void task_adc(void *prm) {
	MEAS meas;
	TASK_ADC_CONF *conf = prm;

	meas.id = conf->id;

	while (1) {
		meas.val = adc_os_read(ADC_OS_MODE_16BIT_SE, conf->adc_chan);
		xQueueSend(conf->meas_q, &meas, 0);
		vTaskDelay(conf->meas_period);
	}
}

void task_tsi(void *prm) {
	MEAS meas;
	TASK_TSI_CONF *conf = prm;

	meas.id = conf->id;

	while (1) {
		meas.val = tsi_os_read(95u);
		if (meas.val != 0u) {
			xQueueSend(conf->meas_q, &meas, 0);
		}
		vTaskDelay(conf->meas_period);
	}
}

void task_prn(void *prm) {
	QueueHandle_t meas_q;
	QueueSetHandle_t *qset = prm;
	MEAS meas;

	char msg[MSG_LEN_LIM];
	uint8_t msg_len;

	while (1) {
		meas_q = xQueueSelectFromSet(*qset, portMAX_DELAY);
		assert(xQueueReceive(meas_q, &meas, 0) == pdPASS);

		msg_len = 0u;

		msg_len += strcpy(&msg[msg_len], TERM_CU_OFF);

		switch (meas.id) {
		case TASK_BGAP_MEAS_ID:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(1,1));
			msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_ULINE_ON));
			msg_len += strcpy(&msg[msg_len], "bgap:");
			break;
		case TASK_TEMP_MEAS_ID:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(2,1));
			msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_ULINE_ON));
			msg_len += strcpy(&msg[msg_len], "temp:");
			break;
		case TASK_TOUCH_MEAS_ID:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(3,1));
			msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_ULINE_ON));
			msg_len += strcpy(&msg[msg_len], "touch:");
			break;
		default:
			msg_len += strcpy(&msg[msg_len], TERM_CUP(4,1));
			msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_ULINE_ON));
			msg_len += strcpy(&msg[msg_len], "unknown:");
			break;
		}

		msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_ULINE_OFF));

		msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_FGC_RED));
		msg_len += print_udec(&msg[msg_len], meas.val);
		msg_len += strcpy(&msg[msg_len], TERM_SGR(TERM_SGR_ARG_FGC_WHITE));

		msg_len += strcpy(&msg[msg_len], TERM_EL(TERM_EL_ARG_CUR_TO_END));

		vcom_os_send(msg, msg_len);
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
	TASK_ADC_CONF task_bgap_conf, task_temp_conf;
	TASK_TSI_CONF task_touch_conf;
	QueueSetHandle_t task_prn_qset;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	task_bgap_conf.meas_q = xQueueCreate(TASK_BGAP_MEAS_Q_LEN, sizeof(MEAS));
	task_bgap_conf.meas_period = TASK_BGAP_MEAS_PERIOD;
	task_bgap_conf.adc_chan = ADC_CHAN_BGAP;
	task_bgap_conf.id = TASK_BGAP_MEAS_ID;

	task_temp_conf.meas_q = xQueueCreate(TASK_TEMP_MEAS_Q_LEN, sizeof(MEAS));
	task_temp_conf.meas_period = TASK_TEMP_MEAS_PERIOD;
	task_temp_conf.adc_chan = ADC_CHAN_TEMP;
	task_temp_conf.id = TASK_TEMP_MEAS_ID;

	task_touch_conf.meas_q = xQueueCreate(TASK_TOUCH_MEAS_Q_LEN, sizeof(MEAS));
	task_touch_conf.meas_period = TASK_TOUCH_MEAS_PERIOD;
	task_touch_conf.id = TASK_TOUCH_MEAS_ID;

	vQueueAddToRegistry(task_bgap_conf.meas_q, "bgap_meas_q");
	vQueueAddToRegistry(task_temp_conf.meas_q, "temp_meas_q");
	vQueueAddToRegistry(task_touch_conf.meas_q, "touch_meas_q");

	task_prn_qset = xQueueCreateSet(TASK_BGAP_MEAS_Q_LEN + TASK_TEMP_MEAS_Q_LEN + TASK_TOUCH_MEAS_Q_LEN);
	xQueueAddToSet(task_bgap_conf.meas_q, task_prn_qset);
	xQueueAddToSet(task_temp_conf.meas_q, task_prn_qset);
	xQueueAddToSet(task_touch_conf.meas_q, task_prn_qset);

	vcom_os_init(NULL, 0u);
	adc_os_init(0u);
	tsi_os_init(0u);

	assert(xTaskCreate(task_adc, "task_bgap", configMINIMAL_STACK_SIZE, &task_bgap_conf, TASK_BGAP_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_adc, "task_temp", configMINIMAL_STACK_SIZE, &task_temp_conf, TASK_TEMP_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_tsi, "task_touch", configMINIMAL_STACK_SIZE, &task_touch_conf, TASK_TOUCH_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_prn, "task_prn", configMINIMAL_STACK_SIZE, &task_prn_qset, TASK_PRN_PRI, NULL) == pdPASS);

	assert(xTaskCreate(task_wdog, "task_wdog", configMINIMAL_STACK_SIZE, NULL, TASK_WDOG_PRI, NULL) == pdPASS);

	vTaskStartScheduler();

	return 0;
}
