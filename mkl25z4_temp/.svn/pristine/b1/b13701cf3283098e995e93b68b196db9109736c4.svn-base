/*
 * Name: btn.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "btn.h"

#include "soc_def.h"
#include "res_alloc.h"
#include "assert.h"

void btn_init(void)
{
	FGPIO_BTN4->PDDR &= ~IOMASK_BTN4;
	FGPIO_BTN3->PDDR &= ~IOMASK_BTN3;
	FGPIO_BTN2->PDDR &= ~IOMASK_BTN2;
	FGPIO_BTN1->PDDR &= ~IOMASK_BTN1;

	PORT_BTN4->PCR[IOIND_BTN4] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
	PORT_BTN3->PCR[IOIND_BTN3] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
	PORT_BTN2->PCR[IOIND_BTN2] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
	PORT_BTN1->PCR[IOIND_BTN1] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
}

uint8_t btn_get(uint8_t id)
{
	uint8_t state = BTN_STATE_PUSHED;

	assert((id >= BTN_ID1) && (id <= BTN_ID4));

	switch (id) {
	case BTN_ID4:

		if (FGPIO_BTN4->PDIR & IOMASK_BTN4) {
			state = BTN_STATE_RELEASED;
		}

		break;
	case BTN_ID3:

		if (FGPIO_BTN3->PDIR & IOMASK_BTN3) {
			state = BTN_STATE_RELEASED;
		}

		break;
	case BTN_ID2:

		if (FGPIO_BTN2->PDIR & IOMASK_BTN2) {
			state = BTN_STATE_RELEASED;
		}

		break;
	case BTN_ID1:

		if (FGPIO_BTN1->PDIR & IOMASK_BTN1) {
			state = BTN_STATE_RELEASED;
		}

		break;
	default:
		break;
	}

	return state;
}
