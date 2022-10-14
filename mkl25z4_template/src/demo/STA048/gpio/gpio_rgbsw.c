/*
 * Name: gpio_rgbsw.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "wdog.h"

typedef enum {
	COLOR_RED,
	COLOR_GREEN,
	COLOR_BLUE
} COLOR;

int main(void)
{
	uint32_t btn_state_old, btn_state_new;
	COLOR color;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	//ledr init
	GPIOB->PCOR = 1ul << 18u;
	GPIOB->PDDR |= 1ul << 18u;
	PORTB->PCR[18u] = PORT_PCR_MUX(1u);

	//ledg init
	GPIOB->PSOR = 1ul << 19u;
	GPIOB->PDDR |= 1ul << 19u;
	PORTB->PCR[19u] = PORT_PCR_MUX(1u);

	//ledb init
	GPIOD->PSOR = 1ul << 1u;
	GPIOD->PDDR |= 1ul << 1u;
	PORTD->PCR[1u] = PORT_PCR_MUX(1u);

	//btn_init
	PORTA->PCR[4u] = PORT_PCR_MUX(1u);

	color = COLOR_RED;
	btn_state_old = GPIOA->PDIR & (1ul << 4u);

	while (1) {
		btn_state_new = GPIOA->PDIR & (1ul << 4u);

		//edge detection
		if ((btn_state_old != 0ul) && (btn_state_new == 0ul)) {

			switch (color) {
			case COLOR_RED:
				color = COLOR_GREEN;
				GPIOB->PSOR = 1ul << 18u;
				GPIOB->PCOR = 1ul << 19u;
				break;
			case COLOR_GREEN:
				color = COLOR_BLUE;
				GPIOB->PSOR = 1ul << 19u;
				GPIOD->PCOR = 1ul << 1u;
				break;
			case COLOR_BLUE:
				color = COLOR_RED;
				GPIOD->PSOR = 1ul << 1u;
				GPIOB->PCOR = 1ul << 18u;
				break;
			default:
				break;
			}

		}

		btn_state_old = btn_state_new;

		wdog_refresh();
	}

	return 0;
}
