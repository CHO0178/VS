/*
 * Name: rtos_uart_alt.c
 * Author: Martin Stankus
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "wdog.h"
#include "vcom_os.h"

#define TASK_SEND_PRI			(configMAX_PRIORITIES - 1)
#define TASK_WDOG_PRI			1

#define BUF_LEN					10u
#define MSG_LIM					4u

void task_send(void *prm) {
	QueueHandle_t *data_que = prm;

	char rx_data;
	char buf[BUF_LEN];
	uint8_t buf_ind = 0u;

	while (1) {
		assert(xQueueReceive(*data_que, &rx_data, portMAX_DELAY) == pdPASS);

		//islower
		if ((rx_data >= 'a') && (rx_data <= 'z')) {

			//toupper
			buf[buf_ind++] = rx_data - 'a' + 'A';

			if (buf_ind == MSG_LIM) {
				buf[buf_ind++] = ' ';
				vcom_os_send(buf, buf_ind);
				buf_ind = 0u;
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
	QueueHandle_t data_que;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	data_que = xQueueCreate(128u, sizeof(uint8_t));

	vcom_os_init(data_que, 0u);

	assert(xTaskCreate(task_send, "task_send", configMINIMAL_STACK_SIZE, &data_que, TASK_SEND_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_wdog, "task_wdog", configMINIMAL_STACK_SIZE, NULL, TASK_WDOG_PRI, NULL) == pdPASS);

	vTaskStartScheduler();

	return 0;
}
