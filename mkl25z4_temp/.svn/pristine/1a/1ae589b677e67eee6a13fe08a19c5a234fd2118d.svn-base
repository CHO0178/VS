/*
 * Name: iic_fram_wr.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "led.h"
#include "wdog.h"
#include "iic_bm.h"
#include "assert.h"

#define FRAM_BUS_ADDR		IIC_ADDR_FRAM_DEV0
#define FRAM_SIZE			32768u

int main(void)
{
	uint32_t ind;

	wdog_init(WDOG_CONF_DIS);

	led_rgb_init();

	PORT_I2C1_SCL_SENS->PCR[IOIND_I2C1_SCL_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
	PORT_I2C1_SDA_SENS->PCR[IOIND_I2C1_SDA_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	assert(iic_recovery(GPIO_I2C1_SCL_SENS, GPIO_I2C1_SDA_SENS, IOMASK_I2C1_SCL_SENS, IOMASK_I2C1_SDA_SENS));

	PORT_I2C1_SCL_SENS->PCR[IOIND_I2C1_SCL_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT6);
	PORT_I2C1_SDA_SENS->PCR[IOIND_I2C1_SDA_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT6);

	iic_init(I2C1, IIC_ICR_VAL_400KHZ);

	for (ind = 0ul; ind < FRAM_SIZE; ind++) {
		iic_bus_start(I2C1);

		assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(FRAM_BUS_ADDR)));

		assert(!iic_bus_tx(I2C1, ind >> 8u));
		assert(!iic_bus_tx(I2C1, ind));

		assert(!iic_bus_tx(I2C1, 0u));

		iic_bus_stop(I2C1);
	}

	led_rgb_set(LED_ID_RED, 100u);

	while (1) {
		__BKPT();
	}

	return 0;
}
