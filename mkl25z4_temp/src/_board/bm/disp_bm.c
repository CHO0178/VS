/*
 * Name: disp_bm.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "disp_bm.h"
#include "iic_bm.h"
#include "assert.h"

void disp_init(void)
{
	uint16_t iter;

	PORT_I2C0_SCL_DISP->PCR[IOIND_I2C0_SCL_DISP] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
	PORT_I2C0_SDA_DISP->PCR[IOIND_I2C0_SDA_DISP] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	assert(iic_recovery(GPIO_I2C0_SCL_DISP, GPIO_I2C0_SDA_DISP, IOMASK_I2C0_SCL_DISP, IOMASK_I2C0_SDA_DISP));

	PORT_I2C0_SCL_DISP->PCR[IOIND_I2C0_SCL_DISP] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT2);
	PORT_I2C0_SDA_DISP->PCR[IOIND_I2C0_SDA_DISP] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT2);

	iic_init(I2C0, IIC_ICR_VAL_400KHZ);

	iic_bus_start(I2C0);

	assert(!iic_bus_tx(I2C0, IIC_MAKE_ADDR_WR(IIC_ADDR_DISP)));
	assert(!iic_bus_tx(I2C0, 0x00u));

	//disable display
	assert(!iic_bus_tx(I2C0, 0xAEu));

	//set vertical addressing mode
	assert(!iic_bus_tx(I2C0, 0x20u));
	assert(!iic_bus_tx(I2C0, 0x01u));

	//set column address
	assert(!iic_bus_tx(I2C0, 0x21u));
	assert(!iic_bus_tx(I2C0, 0x00u));
	assert(!iic_bus_tx(I2C0, DISP_COLUMN_CNT - 1u));

	//set page address
	assert(!iic_bus_tx(I2C0, 0x22u));
	assert(!iic_bus_tx(I2C0, 0x00u));
	assert(!iic_bus_tx(I2C0, DISP_PAGE_CNT - 1u));
	iic_bus_stop(I2C0);

	iic_bus_start(I2C0);

	assert(!iic_bus_tx(I2C0, IIC_MAKE_ADDR_WR(IIC_ADDR_DISP)));
	assert(!iic_bus_tx(I2C0, 0x40u));

	//erase display
	for (iter = 0u; iter < (DISP_PAGE_CNT * DISP_COLUMN_CNT); iter++) {
		assert(!iic_bus_tx(I2C0, 0x00u));
	}

	iic_bus_stop(I2C0);

	iic_bus_start(I2C0);

	assert(!iic_bus_tx(I2C0, IIC_MAKE_ADDR_WR(IIC_ADDR_DISP)));
	assert(!iic_bus_tx(I2C0, 0x00u));

	//enable display
	assert(!iic_bus_tx(I2C0, 0xAFu));

	iic_bus_stop(I2C0);
}

void disp_draw(uint8_t y0, uint8_t y1, uint8_t y2, uint8_t y3)
{
	static uint8_t col_addr = 0u;
	uint8_t col_buf[DISP_PAGE_CNT] = {0u};
	uint8_t iter;

	if (y0 < DISP_PIX_PER_PAGE) {
		y0 = DISP_PIX_PER_PAGE - y0 - 1u;
		col_buf[y0 >> 3u] = 1u << (y0 & 0x7u);
	}

	if (y1 < DISP_PIX_PER_PAGE) {
		y1 = DISP_PIX_PER_PAGE - y1 - 1u;
		col_buf[y1 >> 3u] |= 1u << (y1 & 0x7u);
	}

	if (y2 < DISP_PIX_PER_PAGE) {
		y2 = DISP_PIX_PER_PAGE - y2 - 1u;
		col_buf[y2 >> 3u] |= 1u << (y2 & 0x7u);
	}

	if (y3 < DISP_PIX_PER_PAGE) {
		y3 = DISP_PIX_PER_PAGE - y3 - 1u;
		col_buf[y3 >> 3u] |= 1u << (y3 & 0x7u);
	}

	iic_bus_start(I2C0);

	assert(!iic_bus_tx(I2C0, IIC_MAKE_ADDR_WR(IIC_ADDR_DISP)));
	assert(!iic_bus_tx(I2C0, 0x40u));

	//write signal values
	for (iter = 0u; iter < DISP_PAGE_CNT; iter++) {
		assert(!iic_bus_tx(I2C0, col_buf[iter]));
	}

	//write vertical marker
	if (col_addr < (DISP_COLUMN_CNT - 1u)) {
		for (iter = 0u; iter < DISP_PAGE_CNT; iter++) {
			assert(!iic_bus_tx(I2C0, 0xFFu));
		}
	}

	iic_bus_stop(I2C0);

	col_addr++;
	if (col_addr == DISP_COLUMN_CNT) {
		col_addr = 0u;
	}

	iic_bus_start(I2C0);

	assert(!iic_bus_tx(I2C0, IIC_MAKE_ADDR_WR(IIC_ADDR_DISP)));
	assert(!iic_bus_tx(I2C0, 0x00u));

	//set column address
	assert(!iic_bus_tx(I2C0, 0x21u));
	assert(!iic_bus_tx(I2C0, col_addr));
	assert(!iic_bus_tx(I2C0, DISP_COLUMN_CNT - 1u));

	iic_bus_stop(I2C0);
}
