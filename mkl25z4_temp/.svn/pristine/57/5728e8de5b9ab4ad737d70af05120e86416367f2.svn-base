/*
 * Name: spi_acc_dma.c
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

#define PORTD_EXPT_PRI				2u

#define DMA_UART1_RX_EXPT_PRI		2u
#define DMA_UART1_TX_EXPT_PRI		2u
#define DMA_SPI1_RX_EXPT_PRI		2u

#define DMA_UART1_RX_CHAN			0u
#define DMA_UART1_TX_CHAN			1u
#define DMA_SPI1_RX_CHAN			2u
#define DMA_SPI1_TX_CHAN			3u

#define DMA_UART1_RX_IRQN			DMA0_IRQn
#define DMA_UART1_TX_IRQN			DMA1_IRQn
#define DMA_SPI1_RX_IRQN			DMA2_IRQn
#define DMA_SPI1_TX_IRQN			DMA3_IRQn

#define SPI_RATE_DIV_SPPR			5u
#define SPI_RATE_DIV_SPR			2u

#define ACC_RANGE					ACC_XYZDATCFG_4G
#define ACC_SAMPLE_RATE				ACC_CR1_050_00HZ

#define EXPT_CNT_MOD				50u

#define TX_MSG_LEN_LIM				32u

volatile char uart_rx_buf;
volatile char uart_tx_buf[UART_TX_BUF_LIM];

volatile uint8_t uart_tx_busy = 0;

volatile ACC_SPI_BULK_SAMP_BLK_RX samp_blk_rx;
volatile ACC_SPI_BULK_SAMP_BLK_TX samp_blk_tx;

uint16_t expt_cnt, disp_val;

void uart_enable(void);
void uart_print(volatile ACC_SAMP_BLK *samp);

void acc_enable(void);

uint8_t acc_reg_rd(uint8_t addr);
void acc_reg_wr(uint8_t addr, uint8_t val);
void acc_samp_blk_rd_init(void);

void acc_spi_ctrl_wr(uint8_t ctrl0, uint8_t ctrl1);
uint8_t acc_spi_trailer_rdwr(uint8_t data_out);

void __attribute__ ((interrupt)) PORTD_IRQHandler(void)
{
	PORT_ACC_DOCK_INT1->ISFR = MASK_ACC_DOCK_INT1;

	acc_samp_blk_rd_init();
}

void __attribute__ ((interrupt)) DMA_IRQHandler(DMA_CHAN_UART1_RX)(void)
{
	DMA0->DMA[DMA_CHAN_UART1_RX].DSR_BCR = DMA_DSR_BCR_DONE_MASK;

	//uart_rx_buf contains received data

	DMA0->DMA[DMA_CHAN_UART1_RX].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(uart_rx_buf));
	DMA0->DMA[DMA_CHAN_UART1_RX].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK |
			DMA_DCR_SSIZE(1) | DMA_DCR_DSIZE(1) | DMA_DCR_D_REQ_MASK;
}

void __attribute__ ((interrupt)) DMA_IRQHandler(DMA_CHAN_UART1_TX)(void)
{
	DMA0->DMA[DMA_CHAN_UART1_TX].DSR_BCR = DMA_DSR_BCR_DONE_MASK;
	uart_tx_busy = 0;
}

void __attribute__ ((interrupt)) DMA_IRQHandler(DMA_CHAN_SPI1_RX)(void)
{
	DMA0->DMA[DMA_CHAN_SPI1_RX].DSR_BCR = DMA_DSR_BCR_DONE_MASK;
	DMA0->DMA[DMA_CHAN_SPI1_TX].DSR_BCR = DMA_DSR_BCR_DONE_MASK;

	FGPIO_SPI1_PCS0_ACC->PSOR = MASK_SPI1_PCS0_ACC;

	uart_print(&samp_blk_rx.samp_blk);

	expt_cnt++;
	if (expt_cnt == EXPT_CNT_MOD) {
		expt_cnt = 0;
		led_bindisp(++disp_val);
	}
}

void uart_enable(void)
{
	PORT_UART1_RX->PCR[IOIND_UART1_RX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_UART1_TX->PCR[IOIND_UART1_TX] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	UART1->BDH = UART_BDH_SBR(UART_SBR_115200BD_CLK24M >> 8u);
	UART1->BDL = UART_BDL_SBR(UART_SBR_115200BD_CLK24M);
	UART1->C4 = UART_C4_TDMAS_MASK | UART_C4_RDMAS_MASK;
	UART1->C2 = UART_C2_TIE_MASK | UART_C2_RIE_MASK | UART_C2_TE_MASK |  UART_C2_RE_MASK;

	NVIC_SetPriority(DMA_UART1_RX_IRQN, DMA_UART1_RX_EXPT_PRI);
	NVIC_EnableIRQ(DMA_UART1_RX_IRQN);

	DMA0->DMA[DMA_CHAN_UART1_RX].SAR = (uint32_t) &UART1->D;
	DMA0->DMA[DMA_CHAN_UART1_RX].DAR = (uint32_t) &uart_rx_buf;
	DMA0->DMA[DMA_CHAN_UART1_RX].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(uart_rx_buf));
	DMA0->DMA[DMA_CHAN_UART1_RX].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK |
			DMA_DCR_SSIZE(1) | DMA_DCR_DSIZE(1) | DMA_DCR_D_REQ_MASK;

	NVIC_SetPriority(DMA_UART1_RX_IRQN, DMA_UART1_TX_EXPT_PRI);
	NVIC_EnableIRQ(DMA_UART1_RX_IRQN);

	DMA0->DMA[DMA_CHAN_UART1_TX].DAR = (uint32_t) &UART1->D;

	DMAMUX0->CHCFG[DMA_UART1_RX_CHAN] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_UART1_RX);
	DMAMUX0->CHCFG[DMA_UART1_TX_CHAN] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_UART1_TX);
}

void uart_print(volatile ACC_SAMP_BLK *samp)
{
	uint8_t tx_msg_len;

	if (!uart_tx_busy) {

		tx_msg_len = 0u;
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], TERM_CUP(1,1));
		tx_msg_len += print_uhex(&tx_msg[tx_msg_len], 4u, (((uint16_t) samp->axis_x_hi) << 8u) | samp->axis_x_lo);
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], " ");
		tx_msg_len += print_uhex(&tx_msg[tx_msg_len], 4u, (((uint16_t) samp->axis_y_hi) << 8u) | samp->axis_y_lo);
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], " ");
		tx_msg_len += print_uhex(&tx_msg[tx_msg_len], 4u, (((uint16_t) samp->axis_z_hi) << 8u) | samp->axis_z_lo);
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], TERM_CU_OFF);

		DMA0->DMA[DMA_UART1_TX_CHAN].SAR = (uint32_t) uart_tx_buf;
		DMA0->DMA[DMA_UART1_TX_CHAN].DSR_BCR = DMA_DSR_BCR_BCR(tx_msg_len);
		DMA0->DMA[DMA_UART1_TX_CHAN].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
				DMA_DCR_SINC_MASK | DMA_DCR_SSIZE(1u) | DMA_DCR_DSIZE(1u) | DMA_DCR_D_REQ_MASK;

		uart_tx_busy = 1u;
	}
}

void acc_enable(void)
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

	SPI1->C1 = 0;
	SPI1->C2 = SPI_C2_TXDMAE_MASK | SPI_C2_RXDMAE_MASK;
	SPI1->C1 = SPI_C1_SPE_MASK | SPI_C1_MSTR_MASK;

	samp_blk_tx.ctrl0 = ACC_SPI_MAKE_CTRL0_RD(ACC_ADDR_SAMP_BLK);
	samp_blk_tx.ctrl1 = ACC_SPI_MAKE_CTRL1(ACC_ADDR_SAMP_BLK);

	NVIC_SetPriority(DMA_SPI1_RX_IRQN, DMA_SPI1_RX_EXPT_PRI);
	NVIC_EnableIRQ(DMA_SPI1_RX_IRQN);

	DMA0->DMA[DMA_SPI1_RX_CHAN].SAR = (uint32_t) &SPI1->D;
	DMA0->DMA[DMA_SPI1_TX_CHAN].DAR = (uint32_t) &SPI1->D;

	DMAMUX0->CHCFG[DMA_SPI1_RX_CHAN] = DMAMUX_CHCFG_ENBL_MASK |
			DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_SPI1_RX);
	DMAMUX0->CHCFG[DMA_SPI1_TX_CHAN] = DMAMUX_CHCFG_ENBL_MASK |
			DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_SPI1_TX);

	NVIC_SetPriority(PORTD_IRQn, PORTD_EXPT_PRI);
	NVIC_EnableIRQ(PORTD_IRQn);
}

uint8_t acc_reg_rd(uint8_t addr)
{
	uint8_t data_in;

	FGPIO_ACC_DOCK_SPI1_PCS0->PCOR = IOMASK_ACC_DOCK_SPI1_PCS0;

	acc_spi_ctrl_wr(ACC_SPI_MAKE_CTRL0_RD(addr), ACC_SPI_MAKE_CTRL1(addr));
	data_in = acc_spi_trailer_rdwr(0u);

	FGPIO_ACC_DOCK_SPI1_PCS0->PSOR = IOMASK_ACC_DOCK_SPI1_PCS0;

	return data_in;
}

void acc_reg_wr(uint8_t addr, uint8_t val)
{
	FGPIO_ACC_DOCK_SPI1_PCS0->PCOR = IOMASK_ACC_DOCK_SPI1_PCS0;

	acc_spi_ctrl_wr(ACC_SPI_MAKE_CTRL0_WR(addr), ACC_SPI_MAKE_CTRL1(addr));
	acc_spi_trailer_rdwr(val);

	FGPIO_ACC_DOCK_SPI1_PCS0->PSOR = IOMASK_ACC_DOCK_SPI1_PCS0;
}

void acc_samp_blk_rd_init(void)
{
	FGPIO_ACC_DOCK_SPI1_PCS0->PCOR = IOMASK_ACC_DOCK_SPI1_PCS0;

	DMA0->DMA[DMA_SPI1_RX_CHAN].DAR = (uint32_t) &samp_blk_rx;
	DMA0->DMA[DMA_SPI1_RX_CHAN].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(samp_blk_rx));
	DMA0->DMA[DMA_SPI1_RX_CHAN].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_SSIZE(1u) | DMA_DCR_DINC_MASK | DMA_DCR_DSIZE(1u) | DMA_DCR_D_REQ_MASK;

	DMA0->DMA[DMA_SPI1_TX_CHAN].SAR = (uint32_t) &samp_blk_tx;
	DMA0->DMA[DMA_SPI1_TX_CHAN].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(samp_blk_tx));
	DMA0->DMA[DMA_SPI1_TX_CHAN].DCR = DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_SINC_MASK | DMA_DCR_SSIZE(1u) | DMA_DCR_DSIZE(1u) | DMA_DCR_D_REQ_MASK;
}

void acc_spi_ctrl_wr(uint8_t ctrl0, uint8_t ctrl1)
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

uint8_t acc_spi_trailer_rdwr(uint8_t data_out)
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
	uart_enable();
	acc_enable();

	while (1) {
		wdog_refresh();
	}

	return 0;
}
