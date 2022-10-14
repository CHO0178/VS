/*
 * Name: iic_fram_rd.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "iic_bm.h"
#include "assert.h"

#define FRAM_BUS_ADDR		IIC_ADDR_FRAM_DEV0

int main(void)
{
	volatile uint8_t rec_cnt;

	wdog_init(WDOG_CONF_DIS);

	PORT_I2C1_SCL_SENS->PCR[IOIND_I2C1_SCL_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
	PORT_I2C1_SDA_SENS->PCR[IOIND_I2C1_SDA_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	rec_cnt = iic_recovery(GPIO_I2C1_SCL_SENS, GPIO_I2C1_SDA_SENS, IOMASK_I2C1_SCL_SENS, IOMASK_I2C1_SDA_SENS);
	assert(rec_cnt);

	//examine the value of the rec_cnt variable
	__BKPT();

	PORT_I2C1_SCL_SENS->PCR[IOIND_I2C1_SCL_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT6);
	PORT_I2C1_SDA_SENS->PCR[IOIND_I2C1_SDA_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT6);

	iic_init(I2C1, IIC_ICR_VAL_10KHZ);

	iic_bus_start(I2C1);

	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(FRAM_BUS_ADDR)));

	assert(!iic_bus_tx(I2C1, 0u));
	assert(!iic_bus_tx(I2C1, 0u));

	iic_bus_repstart(I2C1);

	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_RD(FRAM_BUS_ADDR)));

	while (1) {
		assert(iic_bus_rx(I2C1, 0u) == 0u);
	}

	return 0;
}
