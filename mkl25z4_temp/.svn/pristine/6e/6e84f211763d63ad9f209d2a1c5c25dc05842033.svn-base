/*
 * Name: rtos_blink.c
 * Author: Martin Stankus
 *
 */

#include "FreeRTOS.h"
#include "task.h"

#include "wdog.h"
#include "led.h"

#define TASK_BLINK_STACK_SZ		64u
#define TASK_BLINK_PRI			0u

//unit: os ticks
#define SLEEP_TIME				200ul

typedef struct {
	uint32_t sleep_time;
	uint8_t led_id;
} TASK_BLINK_PRM;

void task_blink(void *prm)
{
	TASK_BLINK_PRM *blink_prm = prm;
	TaskHandle_t my_hdl = xTaskGetCurrentTaskHandle();

	//compare the value of my_hdl and task_blink_hdl

	while (1) {
		led_rgb_toggle(blink_prm->led_id);
		vTaskDelay(blink_prm->sleep_time);
	}
}

int main(void)
{
	TaskHandle_t task_blink_hdl;
	TASK_BLINK_PRM task_prm;

	wdog_init(WDOG_CONF_DIS);

	led_rgb_init();

	task_prm.sleep_time = SLEEP_TIME;
	task_prm.led_id = LED_ID_RED;

	xTaskCreate(task_blink, "task_blink", TASK_BLINK_STACK_SZ, &task_prm, TASK_BLINK_PRI, &task_blink_hdl);

	vTaskStartScheduler();

	return 0;
}
