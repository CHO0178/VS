/*
 * Name: spi_acc_dma.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"
#include "wdog.h"
#include "clk.h"
#include "led.h"

#include "dma_irq.h"
#include "dmamux.h"

#include "port.h"
#include "io_dock.h"

#include "acc.h"
#include "ascii_conv.h"

#define PORTD_EXPT_PRI				2

#define DMA_EXPT_PRI_UART1_RX		2
#define DMA_EXPT_PRI_UART1_TX		2
#define DMA_EXPT_PRI_SPI1_RX		2

#define DMA_CHAN_UART1_RX			0
#define DMA_CHAN_UART1_TX			1
#define DMA_CHAN_SPI1_RX			2
#define DMA_CHAN_SPI1_TX			3

#define SPI_RATE_DIV_SPPR			5
#define SPI_RATE_DIV_SPR			2

#define UART1_RATE_DIV_HI			0x00
#define UART1_RATE_DIV_LO			0x0D

#define ACC_RANGE					ACC_XYZDATCFG_4G
#define ACC_SAMPLE_RATE				ACC_CR1_050_00HZ

#define ACC_RST_DELAY				48000ul

#define EXPT_CNT_MOD				50

#define UART_TX_BUF_LIM				32

volatile char uart_rx_buf;
volatile char uart_tx_buf[UART_TX_BUF_LIM];

volatile uint8_t uart_tx_busy = 0;

volatile ACC_SPI_BULK_SAMP_BLK_RX samp_blk_rx;
volatile ACC_SPI_BULK_SAMP_BLK_TX samp_blk_tx;

uint16_t expt_cnt = 0;
uint8_t disp_val = 0;

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

	NVIC_SetPriority(DMA_IRQn(DMA_CHAN_UART1_RX), DMA_EXPT_PRI_UART1_RX);
	NVIC_EnableIRQ(DMA_IRQn(DMA_CHAN_UART1_RX));

	DMA0->DMA[DMA_CHAN_UART1_RX].SAR = (uint32_t) &UART1->D;
	DMA0->DMA[DMA_CHAN_UART1_RX].DAR = (uint32_t) &uart_rx_buf;
	DMA0->DMA[DMA_CHAN_UART1_RX].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(uart_rx_buf));
	DMA0->DMA[DMA_CHAN_UART1_RX].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK |
			DMA_DCR_SSIZE(1) | DMA_DCR_DSIZE(1) | DMA_DCR_D_REQ_MASK;

	NVIC_SetPriority(DMA_IRQn(DMA_CHAN_UART1_TX), DMA_EXPT_PRI_UART1_TX);
	NVIC_EnableIRQ(DMA_IRQn(DMA_CHAN_UART1_TX));

	DMA0->DMA[DMA_CHAN_UART1_TX].DAR = (uint32_t) &UART1->D;

	DMAMUX0->CHCFG[DMA_CHAN_UART1_RX] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_UART1_RX);
	DMAMUX0->CHCFG[DMA_CHAN_UART1_TX] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_UART1_TX);
}

void uart_print(volatile ACC_SAMP_BLK *samp)
{
	uint8_t ind = 0;

	if (!uart_tx_busy) {

		uart_tx_buf[ind++] = '\r';
		uart_tx_buf[ind++] = INT_TO_ASCII(samp->axis_x_hi, NIBBLE1);
		uart_tx_buf[ind++] = INT_TO_ASCII(samp->axis_x_hi, NIBBLE0);
		uart_tx_buf[ind++] = INT_TO_ASCII(samp->axis_x_lo, NIBBLE1);
		uart_tx_buf[ind++] = INT_TO_ASCII(samp->axis_x_lo, NIBBLE0);
		uart_tx_buf[ind++] = ' ';
		uart_tx_buf[ind++] = INT_TO_ASCII(samp->axis_y_hi, NIBBLE1);
		uart_tx_buf[ind++] = INT_TO_ASCII(samp->axis_y_hi, NIBBLE0);
		uart_tx_buf[ind++] = INT_TO_ASCII(samp->axis_y_lo, NIBBLE1);
		uart_tx_buf[ind++] = INT_TO_ASCII(samp->axis_y_lo, NIBBLE0);
		uart_tx_buf[ind++] = ' ';
		uart_tx_buf[ind++] = INT_TO_ASCII(samp->axis_z_hi, NIBBLE1);
		uart_tx_buf[ind++] = INT_TO_ASCII(samp->axis_z_hi, NIBBLE0);
		uart_tx_buf[ind++] = INT_TO_ASCII(samp->axis_z_lo, NIBBLE1);
		uart_tx_buf[ind++] = INT_TO_ASCII(samp->axis_z_lo, NIBBLE0);

		uart_tx_buf[ind++] = 0x1B;
		uart_tx_buf[ind++] = '[';
		uart_tx_buf[ind++] = '?';
		uart_tx_buf[ind++] = '2';
		uart_tx_buf[ind++] = '5';
		uart_tx_buf[ind++] = 'l';

		DMA0->DMA[DMA_CHAN_UART1_TX].SAR = (uint32_t) uart_tx_buf;
		DMA0->DMA[DMA_CHAN_UART1_TX].DSR_BCR = DMA_DSR_BCR_BCR(ind);
		DMA0->DMA[DMA_CHAN_UART1_TX].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
				DMA_DCR_SINC_MASK | DMA_DCR_SSIZE(1) | DMA_DCR_DSIZE(1) | DMA_DCR_D_REQ_MASK;

		uart_tx_busy = 1;
	}
}

void acc_enable(void)
{
	PORT_ACC_DOCK_RST->PCR[IND_ACC_DOCK_RST] = PORT_PCR_MUX(PORT_MUX_GPIO);
	FGPIO_ACC_DOCK_RST->PDDR |= MASK_ACC_DOCK_RST;
	FGPIO_ACC_DOCK_RST->PSOR = MASK_ACC_DOCK_RST;

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0].LDVAL = ACC_RST_DELAY - 1;
	PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN_MASK;

	while (!(PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK));
	PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;

	FGPIO_ACC_DOCK_RST->PCOR = MASK_ACC_DOCK_RST;

	while (!(PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK));

	PORT_SPI1_MISO_ACC->PCR[IND_SPI1_MISO_ACC] = PORT_PCR_MUX(PORT_MUX_ALT2) | PORT_PCR_PE_MASK;
	PORT_SPI1_MOSI_ACC->PCR[IND_SPI1_MOSI_ACC] = PORT_PCR_MUX(PORT_MUX_ALT5);
	PORT_SPI1_SCK_ACC->PCR[IND_SPI1_SCK_ACC] = PORT_PCR_MUX(PORT_MUX_ALT2);

	PORT_SPI1_PCS0_ACC->PCR[IND_SPI1_PCS0_ACC] = PORT_PCR_MUX(PORT_MUX_GPIO);
	FGPIO_SPI1_PCS0_ACC->PDDR |= MASK_SPI1_PCS0_ACC;
	FGPIO_SPI1_PCS0_ACC->PSOR = MASK_SPI1_PCS0_ACC;

	PORT_ACC_DOCK_INT1->PCR[IND_ACC_DOCK_INT1] = PORT_PCR_MUX(PORT_MUX_GPIO) | PORT_PCR_IRQC(PORT_EVNT_INT_EDG_FALL);

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

	NVIC_SetPriority(DMA_IRQn(DMA_CHAN_SPI1_RX), DMA_EXPT_PRI_SPI1_RX);
	NVIC_EnableIRQ(DMA_IRQn(DMA_CHAN_SPI1_RX));

	DMA0->DMA[DMA_CHAN_SPI1_RX].SAR = (uint32_t) &SPI1->D;
	DMA0->DMA[DMA_CHAN_SPI1_TX].DAR = (uint32_t) &SPI1->D;

	DMAMUX0->CHCFG[DMA_CHAN_SPI1_RX] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_SPI1_RX);
	DMAMUX0->CHCFG[DMA_CHAN_SPI1_TX] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_SPI1_TX);

	NVIC_SetPriority(PORTD_IRQn, PORTD_EXPT_PRI);
	NVIC_EnableIRQ(PORTD_IRQn);
}

uint8_t acc_reg_rd(uint8_t addr)
{
	uint8_t data_in;

	FGPIO_SPI1_PCS0_ACC->PCOR = MASK_SPI1_PCS0_ACC;

	acc_spi_ctrl_wr(ACC_SPI_MAKE_CTRL0_RD(addr), ACC_SPI_MAKE_CTRL1(addr));
	data_in = acc_spi_trailer_rdwr(0);

	FGPIO_SPI1_PCS0_ACC->PSOR = MASK_SPI1_PCS0_ACC;

	return data_in;
}

void acc_reg_wr(uint8_t addr, uint8_t val)
{
	FGPIO_SPI1_PCS0_ACC->PCOR = MASK_SPI1_PCS0_ACC;

	acc_spi_ctrl_wr(ACC_SPI_MAKE_CTRL0_WR(addr), ACC_SPI_MAKE_CTRL1(addr));
	acc_spi_trailer_rdwr(val);

	FGPIO_SPI1_PCS0_ACC->PSOR = MASK_SPI1_PCS0_ACC;
}

void acc_samp_blk_rd_init(void)
{
	FGPIO_SPI1_PCS0_ACC->PCOR = MASK_SPI1_PCS0_ACC;

	DMA0->DMA[DMA_CHAN_SPI1_RX].DAR = (uint32_t) &samp_blk_rx;
	DMA0->DMA[DMA_CHAN_SPI1_RX].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(samp_blk_rx));
	DMA0->DMA[DMA_CHAN_SPI1_RX].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_SSIZE(1) | DMA_DCR_DINC_MASK | DMA_DCR_DSIZE(1) | DMA_DCR_D_REQ_MASK;

	DMA0->DMA[DMA_CHAN_SPI1_TX].SAR = (uint32_t) &samp_blk_tx;
	DMA0->DMA[DMA_CHAN_SPI1_TX].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(samp_blk_tx));
	DMA0->DMA[DMA_CHAN_SPI1_TX].DCR = DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_SINC_MASK | DMA_DCR_SSIZE(1) | DMA_DCR_DSIZE(1) | DMA_DCR_D_REQ_MASK;
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
	wdog_init(WDOG_CONF_LPO_1024_CYCLES);
	clk_periph_en();
	led_init();

	uart_enable();

	acc_enable();

	while (1) {
		wdog_refresh();
	}

	return 0;
}
