/*
 * Name: rtos_evtg.c
 * Author: Martin Stankus
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "wdog.h"
#include "led.h"
#include "assert.h"

#define TASK_LED1_PRI			(configMAX_PRIORITIES - 1)
#define TASK_LED2_PRI			(configMAX_PRIORITIES - 1)
#define TASK_LED3_PRI			(configMAX_PRIORITIES - 1)
#define TASK_LED4_PRI			(configMAX_PRIORITIES - 1)
#define TASK_LED5_PRI			(configMAX_PRIORITIES - 1)
#define TASK_LED6_PRI			(configMAX_PRIORITIES - 1)
#define TASK_LED7_PRI			(configMAX_PRIORITIES - 1)
#define TASK_LED8_PRI			(configMAX_PRIORITIES - 1)
#define TASK_WDOG_PRI			0

#define TASK_LED1_DELAY			200u
#define TASK_LED2_DELAY			200u
#define TASK_LED3_DELAY			200u
#define TASK_LED4_DELAY			200u
#define TASK_LED5_DELAY			200u
#define TASK_LED6_DELAY			200u
#define TASK_LED7_DELAY			200u
#define TASK_LED8_DELAY			200u

#define TASK_LED1_EVT_MASK		0x01u
#define TASK_LED2_EVT_MASK		0x02u
#define TASK_LED3_EVT_MASK		0x04u
#define TASK_LED4_EVT_MASK		0x08u
#define TASK_LED5_EVT_MASK		0x10u
#define TASK_LED6_EVT_MASK		0x20u
#define TASK_LED7_EVT_MASK		0x40u
#define TASK_LED8_EVT_MASK		0x80u

typedef struct {
	EventGroupHandle_t evtg;
	EventBits_t evtg_mask;
	EventBits_t evtg_mask_next;
	TickType_t delay;
	uint8_t led_id;
} TASK_LED_CONF;

void task_led(void *prm) {
	TASK_LED_CONF *conf = prm;

	while (1) {
		xEventGroupWaitBits(conf->evtg, conf->evtg_mask, pdTRUE, pdFALSE, portMAX_DELAY);

		led_on(conf->led_id);
		vTaskDelay(conf->delay);
		led_off(conf->led_id);

		xEventGroupSetBits(conf->evtg, conf->evtg_mask_next);
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
	EventGroupHandle_t evtg;
	TASK_LED_CONF task_led_conf[8u];

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();

	evtg = xEventGroupCreate();
	assert(evtg != NULL);

	task_led_conf[0u].evtg = evtg;
	task_led_conf[0u].evtg_mask = TASK_LED1_EVT_MASK;
	task_led_conf[0u].evtg_mask_next = TASK_LED2_EVT_MASK;
	task_led_conf[0u].delay = TASK_LED1_DELAY;
	task_led_conf[0u].led_id = LED_ID1;

	task_led_conf[1u].evtg = evtg;
	task_led_conf[1u].evtg_mask = TASK_LED2_EVT_MASK;
	task_led_conf[1u].evtg_mask_next = TASK_LED3_EVT_MASK;
	task_led_conf[1u].delay = TASK_LED2_DELAY;
	task_led_conf[1u].led_id = LED_ID2;

	task_led_conf[2u].evtg = evtg;
	task_led_conf[2u].evtg_mask = TASK_LED3_EVT_MASK;
	task_led_conf[2u].evtg_mask_next = TASK_LED4_EVT_MASK;
	task_led_conf[2u].delay = TASK_LED3_DELAY;
	task_led_conf[2u].led_id = LED_ID3;

	task_led_conf[3u].evtg = evtg;
	task_led_conf[3u].evtg_mask = TASK_LED4_EVT_MASK;
	task_led_conf[3u].evtg_mask_next = TASK_LED5_EVT_MASK;
	task_led_conf[3u].delay = TASK_LED4_DELAY;
	task_led_conf[3u].led_id = LED_ID4;

	task_led_conf[4u].evtg = evtg;
	task_led_conf[4u].evtg_mask = TASK_LED5_EVT_MASK;
	task_led_conf[4u].evtg_mask_next = TASK_LED6_EVT_MASK;
	task_led_conf[4u].delay = TASK_LED5_DELAY;
	task_led_conf[4u].led_id = LED_ID5;

	task_led_conf[5u].evtg = evtg;
	task_led_conf[5u].evtg_mask = TASK_LED6_EVT_MASK;
	task_led_conf[5u].evtg_mask_next = TASK_LED7_EVT_MASK;
	task_led_conf[5u].delay = TASK_LED6_DELAY;
	task_led_conf[5u].led_id = LED_ID6;

	task_led_conf[6u].evtg = evtg;
	task_led_conf[6u].evtg_mask = TASK_LED7_EVT_MASK;
	task_led_conf[6u].evtg_mask_next = TASK_LED8_EVT_MASK;
	task_led_conf[6u].delay = TASK_LED7_DELAY;
	task_led_conf[6u].led_id = LED_ID7;

	task_led_conf[7u].evtg = evtg;
	task_led_conf[7u].evtg_mask = TASK_LED8_EVT_MASK;
	task_led_conf[7u].evtg_mask_next = TASK_LED1_EVT_MASK;
	task_led_conf[7u].delay = TASK_LED8_DELAY;
	task_led_conf[7u].led_id = LED_ID8;

	xEventGroupSetBits(evtg, TASK_LED1_EVT_MASK);

	assert(xTaskCreate(task_led, "task_led1", configMINIMAL_STACK_SIZE, &task_led_conf[0u], TASK_LED1_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_led, "task_led2", configMINIMAL_STACK_SIZE, &task_led_conf[1u], TASK_LED2_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_led, "task_led3", configMINIMAL_STACK_SIZE, &task_led_conf[2u], TASK_LED3_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_led, "task_led4", configMINIMAL_STACK_SIZE, &task_led_conf[3u], TASK_LED4_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_led, "task_led5", configMINIMAL_STACK_SIZE, &task_led_conf[4u], TASK_LED5_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_led, "task_led6", configMINIMAL_STACK_SIZE, &task_led_conf[5u], TASK_LED6_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_led, "task_led7", configMINIMAL_STACK_SIZE, &task_led_conf[6u], TASK_LED7_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_led, "task_led8", configMINIMAL_STACK_SIZE, &task_led_conf[7u], TASK_LED8_PRI, NULL) == pdPASS);

	assert(xTaskCreate(task_wdog, "task_wdog", configMINIMAL_STACK_SIZE, NULL, TASK_WDOG_PRI, NULL) == pdPASS);

	vTaskStartScheduler();

	return 0;
}
