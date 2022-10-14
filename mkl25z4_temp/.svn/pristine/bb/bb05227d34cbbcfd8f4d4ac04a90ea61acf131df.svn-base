/*
 * Name: spi_rd_wr.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "acc.h"
#include "wdog.h"
#include "wait_bm.h"
#include "assert.h"

#define SPI_RATE_DIV_SPPR			5u
#define SPI_RATE_DIV_SPR			2u

#define SPI_ACC_CS_ASSERT()			(GPIO_ACC_DOCK_SPI1_PCS0->PCOR = IOMASK_ACC_DOCK_SPI1_PCS0)
#define SPI_ACC_CS_DEASSERT()		(GPIO_ACC_DOCK_SPI1_PCS0->PSOR = IOMASK_ACC_DOCK_SPI1_PCS0)

static void acc_reset(void);
static void spi_init(void);
static uint8_t spi_bus_rx_tx(uint8_t data_tx);

static void acc_reset(void)
{
	GPIO_ACC_DOCK_RST->PSOR = IOMASK_ACC_DOCK_RST;
	GPIO_ACC_DOCK_RST->PDDR |= IOMASK_ACC_DOCK_RST;
	PORT_ACC_DOCK_RST->PCR[IOIND_ACC_DOCK_RST] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	wait_bm(WAIT_BM_10MSEC_FSYS48M, 1u);

	GPIO_ACC_DOCK_RST->PCOR = IOMASK_ACC_DOCK_RST;

	wait_bm(WAIT_BM_10MSEC_FSYS48M, 1u);
}

static void spi_init(void)
{
	PORT_ACC_DOCK_SPI1_MISO->PCR[IOIND_ACC_DOCK_SPI1_MISO] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT2) | PORT_PCR_PE_MASK;
	PORT_ACC_DOCK_SPI1_MOSI->PCR[IOIND_ACC_DOCK_SPI1_MOSI] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT5);
	PORT_ACC_DOCK_SPI1_SCK->PCR[IOIND_ACC_DOCK_SPI1_SCK] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT2);

	SPI_ACC_CS_DEASSERT();
	GPIO_ACC_DOCK_SPI1_PCS0->PDDR |= IOMASK_ACC_DOCK_SPI1_PCS0;
	PORT_ACC_DOCK_SPI1_PCS0->PCR[IOIND_ACC_DOCK_SPI1_PCS0] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	SPI1->BR = SPI_BR_SPPR(SPI_RATE_DIV_SPPR) | SPI_BR_SPR(SPI_RATE_DIV_SPR);
	SPI1->C1 = SPI_C1_SPE_MASK | SPI_C1_MSTR_MASK;
}

static uint8_t spi_bus_rx_tx(uint8_t data_tx)
{
	SPI1->S;
	SPI1->D = data_tx;

	while (!(SPI1->S & SPI_S_SPRF_MASK));
	return SPI1->D;
}

int main(void)
{
	uint8_t data;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	acc_reset();
	spi_init();

	data = 0xAAu;

	while (1) {

		//write transaction
		SPI_ACC_CS_ASSERT();

		spi_bus_rx_tx(ACC_SPI_MAKE_CTRL0_WR(ACC_ADDR_CR4));
		spi_bus_rx_tx(ACC_SPI_MAKE_CTRL1(ACC_ADDR_CR4));

		spi_bus_rx_tx(data);

		SPI_ACC_CS_DEASSERT();

		wait_bm(WAIT_BM_100USEC_FSYS48M, 1u);

		//read transaction
		SPI_ACC_CS_ASSERT();

		spi_bus_rx_tx(ACC_SPI_MAKE_CTRL0_RD(ACC_ADDR_CR4));
		spi_bus_rx_tx(ACC_SPI_MAKE_CTRL1(ACC_ADDR_CR4));

		assert(spi_bus_rx_tx(0u) == data);

		SPI_ACC_CS_DEASSERT();

		wait_bm(WAIT_BM_2MSEC_FSYS48M, 1u);

		data ^= 0xFF;
	}

	return 0;
}
