/*
 * Name: rtos_evtg_alt.c
 * Author: Martin Stankus
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "wdog.h"
#include "led.h"
#include "assert.h"

#define TASK_LED_RED_PRI			(configMAX_PRIORITIES - 1)
#define TASK_LED_GREEN_PRI			(configMAX_PRIORITIES - 2)
#define TASK_LED_BLUE_PRI			(configMAX_PRIORITIES - 3)
#define TASK_WDOG_PRI				1

#define TASK_LED_RED_DELAY			100u
#define TASK_LED_GREEN_DELAY		200u
#define TASK_LED_BLUE_DELAY			300u

#define TASK_LED_RED_EVT_MASK		0x01u
#define TASK_LED_GREEN_EVT_MASK		0x02u
#define TASK_LED_BLUE_EVT_MASK		0x04u

typedef struct {
	EventGroupHandle_t evtg;
	EventBits_t evtg_mask;
	EventBits_t evtg_mask_next;
	TickType_t delay;
	uint8_t led_id;
} TASK_LED_CONF;

void task_led(void *prm) {
	TASK_LED_CONF *conf = prm;
	TaskHandle_t my_hdl = xTaskGetCurrentTaskHandle();

	while (1) {
		xEventGroupWaitBits(conf->evtg, conf->evtg_mask, pdTRUE, pdFALSE, /*portMAX_DELAY*/10000);

		led_rgb_set(conf->led_id, LED_RGB_PWM_PERIOD / 20u);
		vTaskDelay(conf->delay);
		led_rgb_set(conf->led_id, 0u);

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

	led_rgb_init();

	evtg = xEventGroupCreate();
	assert(evtg != NULL);

	task_led_conf[0u].evtg = evtg;
	task_led_conf[0u].evtg_mask = TASK_LED_RED_EVT_MASK;
	task_led_conf[0u].evtg_mask_next = TASK_LED_GREEN_EVT_MASK;
	task_led_conf[0u].delay = TASK_LED_RED_DELAY;
	task_led_conf[0u].led_id = LED_ID_RED;

	task_led_conf[1u].evtg = evtg;
	task_led_conf[1u].evtg_mask = TASK_LED_GREEN_EVT_MASK;
	task_led_conf[1u].evtg_mask_next = TASK_LED_BLUE_EVT_MASK;
	task_led_conf[1u].delay = TASK_LED_GREEN_DELAY;
	task_led_conf[1u].led_id = LED_ID_GREEN;

	task_led_conf[2u].evtg = evtg;
	task_led_conf[2u].evtg_mask = TASK_LED_BLUE_EVT_MASK;
	task_led_conf[2u].evtg_mask_next = TASK_LED_RED_EVT_MASK;
	task_led_conf[2u].delay = TASK_LED_BLUE_DELAY;
	task_led_conf[2u].led_id = LED_ID_BLUE;

	xEventGroupSetBits(evtg, TASK_LED_RED_EVT_MASK);

	assert(xTaskCreate(task_led, "task_led_red", configMINIMAL_STACK_SIZE, &task_led_conf[0u], TASK_LED_RED_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_led, "task_led_green", configMINIMAL_STACK_SIZE, &task_led_conf[1u], TASK_LED_GREEN_PRI, NULL) == pdPASS);
	assert(xTaskCreate(task_led, "task_led_blue", configMINIMAL_STACK_SIZE, &task_led_conf[2u], TASK_LED_BLUE_PRI, NULL) == pdPASS);

	assert(xTaskCreate(task_wdog, "task_wdog", configMINIMAL_STACK_SIZE, NULL, TASK_WDOG_PRI, NULL) == pdPASS);

	vTaskStartScheduler();

	return 0;
}
