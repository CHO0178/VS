/*
 * Name: rtos_sched_coop.c
 * Author: Martin Stankus
 *
 * FreeRTOSConfig.h ->
 * configUSE_PREEMPTION			0
 * configUSE_IDLE_HOOK			0
 *
 */

#include "FreeRTOS.h"
#include "task.h"

#include "wdog.h"
#include "led.h"

#define TASK_LED1_STACK_SZ		64u
#define TASK_LED1_PRI			0u

#define TASK_LED2_STACK_SZ		64u
#define TASK_LED2_PRI			1u

//unit: iterations
#define BWAIT_TIME				1000000ul

//unit: os ticks
#define SLEEP_TIME				100ul

void task_led2(void *prm) {
	uint32_t bwait;

	while (1) {
		led_toggle(LED_ID2);
		for (bwait = 0ul; bwait < BWAIT_TIME; bwait++);
		//taskYIELD();
		//vTaskDelay(SLEEP_TIME);
	}
}

void task_led1(void *prm) {
	uint32_t bwait;

	xTaskCreate(task_led2, "task_led2", TASK_LED2_STACK_SZ, NULL, TASK_LED2_PRI, NULL);

	while (1) {
		led_toggle(LED_ID1);
		for (bwait = 0ul; bwait < BWAIT_TIME; bwait++);
		//taskYIELD();
		//vTaskDelay(SLEEP_TIME);
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_DIS);

	led_init();

	xTaskCreate(task_led1, "task_led1", TASK_LED1_STACK_SZ, NULL, TASK_LED1_PRI, NULL);

	vTaskStartScheduler();

	return 0;
}
