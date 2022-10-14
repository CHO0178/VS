/*
 * Name: spi_acc.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "led.h"
#include "wait_bm.h"
#include "print.h"
#include "str.h"
#include "term.h"
#include "acc.h"

#define UART1_EXPT_PRI		2u
#define PORTD_EXPT_PRI		2u

#define SPI_RATE_DIV_SPPR	5u
#define SPI_RATE_DIV_SPR	2u

#define ACC_RANGE			ACC_XYZDATCFG_4G
#define ACC_SAMPLE_RATE		ACC_CR1_050_00HZ

#define EXPT_CNT_MOD		50u

#define TX_MSG_LEN_LIM		32u

static char tx_msg[TX_MSG_LEN_LIM];
static volatile uint8_t tx_msg_len, tx_msg_ind;

static uint16_t expt_cnt, disp_val;

static void uart_init(void);
static void uart_print(ACC_SAMP_BLK *samp);

static void acc_enable(void);

static uint8_t acc_reg_rd(uint8_t addr);
static void acc_reg_wr(uint8_t addr, uint8_t val);
static void acc_samp_blk_rd(ACC_SAMP_BLK *samp);

static void acc_spi_ctrl_wr(uint8_t ctrl0, uint8_t ctrl1);
static uint8_t acc_spi_trailer_rdwr(uint8_t data_out);

void __attribute__ ((interrupt)) UART1_IRQHandler(void)
{
	if (UART1->S1 & UART_S1_RDRF_MASK) {
		UART1->D;
	}

	if ((UART1->C2 & UART_C2_TIE_MASK) && (UART1->S1 & UART_S1_TDRE_MASK)) {
		if (tx_msg_ind < tx_msg_len) {
			UART1->D = tx_msg[tx_msg_ind++];
		} else {
			UART1->C2 = UART_C2_RIE_MASK | UART_C2_TE_MASK | UART_C2_RE_MASK;
		}
	}
}

void __attribute__ ((interrupt)) PORTD_IRQHandler(void)
{
	ACC_SAMP_BLK samp_blk;

	PORT_ACC_DOCK_INT1->ISFR = IOMASK_ACC_DOCK_INT1;

	acc_samp_blk_rd(&samp_blk);
	uart_print(&samp_blk);

	expt_cnt++;
	if (expt_cnt == EXPT_CNT_MOD) {
		expt_cnt = 0;
		led_bindisp(++disp_val);
	}
}

static void uart_init(void)
{
	PORT_UART1_RX->PCR[IOIND_UART1_RX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_UART1_TX->PCR[IOIND_UART1_TX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	NVIC_SetPriority(UART1_IRQn, UART1_EXPT_PRI);
	NVIC_EnableIRQ(UART1_IRQn);

	UART1->BDH = UART_BDH_SBR(UART_SBR_115200BD_CLK24M >> 8u);
	UART1->BDL = UART_BDL_SBR(UART_SBR_115200BD_CLK24M);
	UART1->C2 = UART_C2_RIE_MASK | UART_C2_TE_MASK | UART_C2_RE_MASK;
}

static void uart_print(ACC_SAMP_BLK *samp)
{
	if (!(UART1->C2 & UART_C2_TIE_MASK)) {

		tx_msg_len = 0u;
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], TERM_CUP(1,1));
		tx_msg_len += print_uhex(&tx_msg[tx_msg_len], 4u, (((uint16_t) samp->axis_x_hi) << 8u) | samp->axis_x_lo);
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], " ");
		tx_msg_len += print_uhex(&tx_msg[tx_msg_len], 4u, (((uint16_t) samp->axis_y_hi) << 8u) | samp->axis_y_lo);
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], " ");
		tx_msg_len += print_uhex(&tx_msg[tx_msg_len], 4u, (((uint16_t) samp->axis_z_hi) << 8u) | samp->axis_z_lo);
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], TERM_CU_OFF);

		tx_msg_ind = 0u;
		UART1->C2 = UART_C2_TIE_MASK | UART_C2_RIE_MASK | UART_C2_TE_MASK |  UART_C2_RE_MASK;
	}
}

static void acc_enable(void)
{
	FGPIO_ACC_DOCK_RST->PSOR = IOMASK_ACC_DOCK_RST;
	FGPIO_ACC_DOCK_RST->PDDR |= IOMASK_ACC_DOCK_RST;
	PORT_ACC_DOCK_RST->PCR[IOIND_ACC_DOCK_RST] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	wait_bm(WAIT_BM_10MSEC_FSYS48M, 1u);

	FGPIO_ACC_DOCK_RST->PCOR = IOMASK_ACC_DOCK_RST;

	wait_bm(WAIT_BM_10MSEC_FSYS48M, 1u);

	PORT_ACC_DOCK_SPI1_MISO->PCR[IOIND_ACC_DOCK_SPI1_MISO] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT2) |
			PORT_PCR_PE_MASK;
	PORT_ACC_DOCK_SPI1_MOSI->PCR[IOIND_ACC_DOCK_SPI1_MOSI] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT5);
	PORT_ACC_DOCK_SPI1_SCK->PCR[IOIND_ACC_DOCK_SPI1_SCK] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT2);

	FGPIO_ACC_DOCK_SPI1_PCS0->PSOR = IOMASK_ACC_DOCK_SPI1_PCS0;
	FGPIO_ACC_DOCK_SPI1_PCS0->PDDR |= IOMASK_ACC_DOCK_SPI1_PCS0;
	PORT_ACC_DOCK_SPI1_PCS0->PCR[IOIND_ACC_DOCK_SPI1_PCS0] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	PORT_ACC_DOCK_INT1->PCR[IOIND_ACC_DOCK_INT1] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO) |
			PORT_PCR_IRQC(PORT_PCR_IRQC_VAL_INT_EFALL);

	SPI1->BR = SPI_BR_SPPR(SPI_RATE_DIV_SPPR) | SPI_BR_SPR(SPI_RATE_DIV_SPR);
	SPI1->C1 = SPI_C1_SPE_MASK | SPI_C1_MSTR_MASK;

	acc_reg_wr(ACC_ADDR_XYZDATCFG, ACC_RANGE);
	acc_reg_wr(ACC_ADDR_CR4, ACC_CR4_INT_EN_DRDY);
	acc_reg_wr(ACC_ADDR_CR5, ACC_CR5_INT_DRDY_INT1);
	acc_reg_wr(ACC_ADDR_CR1, ACC_SAMPLE_RATE);

	NVIC_SetPriority(PORTD_IRQn, PORTD_EXPT_PRI);
	NVIC_EnableIRQ(PORTD_IRQn);
}

static uint8_t acc_reg_rd(uint8_t addr)
{
	uint8_t data_in;

	FGPIO_ACC_DOCK_SPI1_PCS0->PCOR = IOMASK_ACC_DOCK_SPI1_PCS0;

	acc_spi_ctrl_wr(ACC_SPI_MAKE_CTRL0_RD(addr), ACC_SPI_MAKE_CTRL1(addr));
	data_in = acc_spi_trailer_rdwr(0u);

	FGPIO_ACC_DOCK_SPI1_PCS0->PSOR = IOMASK_ACC_DOCK_SPI1_PCS0;

	return data_in;
}

static void acc_reg_wr(uint8_t addr, uint8_t val)
{
	FGPIO_ACC_DOCK_SPI1_PCS0->PCOR = IOMASK_ACC_DOCK_SPI1_PCS0;

	acc_spi_ctrl_wr(ACC_SPI_MAKE_CTRL0_WR(addr), ACC_SPI_MAKE_CTRL1(addr));
	acc_spi_trailer_rdwr(val);

	FGPIO_ACC_DOCK_SPI1_PCS0->PSOR = IOMASK_ACC_DOCK_SPI1_PCS0;
}

static void acc_samp_blk_rd(ACC_SAMP_BLK *samp_blk)
{
	FGPIO_ACC_DOCK_SPI1_PCS0->PCOR = IOMASK_ACC_DOCK_SPI1_PCS0;

	acc_spi_ctrl_wr(ACC_SPI_MAKE_CTRL0_RD(ACC_ADDR_SAMP_BLK), ACC_SPI_MAKE_CTRL1(ACC_ADDR_SAMP_BLK));
	samp_blk->axis_x_hi = acc_spi_trailer_rdwr(0u);
	samp_blk->axis_x_lo = acc_spi_trailer_rdwr(0u);
	samp_blk->axis_y_hi = acc_spi_trailer_rdwr(0u);
	samp_blk->axis_y_lo = acc_spi_trailer_rdwr(0u);
	samp_blk->axis_z_hi = acc_spi_trailer_rdwr(0u);
	samp_blk->axis_z_lo = acc_spi_trailer_rdwr(0u);

	FGPIO_ACC_DOCK_SPI1_PCS0->PSOR = IOMASK_ACC_DOCK_SPI1_PCS0;
}

static void acc_spi_ctrl_wr(uint8_t ctrl0, uint8_t ctrl1)
{
	SPI1->S;
	SPI1->D = ctrl0;

	while (!(SPI1->S & SPI_S_SPRF_MASK));
	SPI1->D;

	SPI1->S;
	SPI1->D = ctrl1;

	while (!(SPI1->S & SPI_S_SPRF_MASK));
	SPI1->D;
}

static uint8_t acc_spi_trailer_rdwr(uint8_t data_out)
{
	SPI1->S;
	SPI1->D = data_out;

	while (!(SPI1->S & SPI_S_SPRF_MASK));
	return SPI1->D;
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();
	uart_init();

	acc_enable();

	while (1) {
		wdog_refresh();
	}

	return 0;
}
