/*
 * Name: cube_slave.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "wdog.h"
#include "bme.h"

#define DAC0_DAT0		(*((volatile uint16_t *) 0x4003F000ul))

int main(void)
{
	uint16_t buf, dac_data;

	wdog_init(WDOG_CONF_DIS);

	DAC0->C0 = DAC_C0_DACEN_MASK;

	SPI0->C1 = SPI_C1_SPE_MASK;

	PORTD->PCR[0] = PORT_PCR_MUX(2);
	PORTD->PCR[1] = PORT_PCR_MUX(2);
	PORTD->PCR[2] = PORT_PCR_MUX(2);
	PORTD->PCR[3] = PORT_PCR_MUX(2);

	while (1) {

		while (!BME_UBFX_B(&SPI0->S, SPI_S_SPRF_SHIFT, 1));
		buf = SPI0->D;

		if (buf & 0x80) {
			dac_data = buf << 6;
		} else {
			dac_data |= buf;
			DAC0_DAT0 = dac_data;
		}

	}

	return 0;
}
