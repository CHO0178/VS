/*
 * Name: iic_fram_rd_wr.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "iic_bm.h"
#include "wait_bm.h"
#include "assert.h"

#define FRAM_BUS_ADDR		IIC_ADDR_FRAM_DEV0
#define FRAM_DATA_ADDR		0x6234u

int main(void)
{
	uint8_t data[4u];

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	PORT_I2C1_SCL_SENS->PCR[IOIND_I2C1_SCL_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
	PORT_I2C1_SDA_SENS->PCR[IOIND_I2C1_SDA_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	assert(iic_recovery(GPIO_I2C1_SCL_SENS, GPIO_I2C1_SDA_SENS, IOMASK_I2C1_SCL_SENS, IOMASK_I2C1_SDA_SENS));

	PORT_I2C1_SCL_SENS->PCR[IOIND_I2C1_SCL_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT6);
	PORT_I2C1_SDA_SENS->PCR[IOIND_I2C1_SDA_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT6);

	iic_init(I2C1, IIC_ICR_VAL_400KHZ);

	data[0u] = 0x55u;
	data[1u] = 0x00u;
	data[2u] = 0xFFu;
	data[3u] = 0xAAu;

	while (1) {

		//write transaction
		iic_bus_start(I2C1);

		assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(FRAM_BUS_ADDR)));

		assert(!iic_bus_tx(I2C1, FRAM_DATA_ADDR >> 8u));
		assert(!iic_bus_tx(I2C1, (uint8_t) FRAM_DATA_ADDR));

		assert(!iic_bus_tx(I2C1, data[0u]));
		assert(!iic_bus_tx(I2C1, data[1u]));
		assert(!iic_bus_tx(I2C1, data[2u]));
		assert(!iic_bus_tx(I2C1, data[3u]));

		iic_bus_stop(I2C1);

		wait_bm(WAIT_BM_250USEC_FSYS48M, 1u);

		//read transaction
		iic_bus_start(I2C1);

		assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(FRAM_BUS_ADDR)));

		assert(!iic_bus_tx(I2C1, FRAM_DATA_ADDR >> 8u));
		assert(!iic_bus_tx(I2C1, (uint8_t) FRAM_DATA_ADDR));

		iic_bus_repstart(I2C1);

		assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_RD(FRAM_BUS_ADDR)));

		assert(iic_bus_rx(I2C1, 0u) == data[0u]);
		assert(iic_bus_rx(I2C1, 0u) == data[1u]);
		assert(iic_bus_rx(I2C1, 0u) == data[2u]);
		assert(iic_bus_rx(I2C1, 1u) == data[3u]);

		iic_bus_stop(I2C1);

		wait_bm(WAIT_BM_2MSEC_FSYS48M, 1u);

		data[0u] ^= 0xFF;
		data[1u] ^= 0xFF;
		data[2u] ^= 0xFF;
		data[3u] ^= 0xFF;
	}

	return 0;
}
