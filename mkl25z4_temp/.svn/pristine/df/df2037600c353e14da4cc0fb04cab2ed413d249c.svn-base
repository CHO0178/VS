/*
 * Name: dma_basic.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "led.h"

#define DMA_CHAN0_EXPT_PRI		2u
#define DMA_CHAN1_EXPT_PRI		2u
#define DMA_CHAN2_EXPT_PRI		2u
#define DMA_CHAN3_EXPT_PRI		2u

#define MEM_BLK_LEN				4u

volatile uint32_t memcpy_src[MEM_BLK_LEN] = {0x11223344ul, 0x55667788ul, 0x99AABBCCul, 0xDDEEFFA5ul};
volatile uint32_t memcpy_dst_cont[MEM_BLK_LEN] = {0ul};
volatile uint32_t memcpy_dst_req[MEM_BLK_LEN] = {0ul};

volatile uint32_t memset_src = 0x5A5A5A5Aul;
volatile uint32_t memset_dst_cont[MEM_BLK_LEN] = {0ul};
volatile uint32_t memset_dst_req[MEM_BLK_LEN] = {0ul};

void __attribute__ ((interrupt)) DMA0_IRQHandler(void)
{
	DMA0->DMA[0u].DSR_BCR = DMA_DSR_BCR_DONE_MASK;
	DMA0->DMA[1u].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_SINC_MASK | DMA_DCR_SSIZE(0u) | DMA_DCR_DINC_MASK | DMA_DCR_DSIZE(0u) |
			DMA_DCR_D_REQ_MASK;
}

void __attribute__ ((interrupt)) DMA1_IRQHandler(void)
{
	DMA0->DMA[1u].DSR_BCR = DMA_DSR_BCR_DONE_MASK;
	DMA0->DMA[2u].DCR = DMA_DCR_EINT_MASK |
			DMA_DCR_SSIZE(0u) | DMA_DCR_DINC_MASK | DMA_DCR_DSIZE(0u) |
			DMA_DCR_START_MASK;
}

void __attribute__ ((interrupt)) DMA2_IRQHandler(void)
{
	DMA0->DMA[2u].DSR_BCR = DMA_DSR_BCR_DONE_MASK;
	DMA0->DMA[3u].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_SSIZE(0u) | DMA_DCR_DINC_MASK | DMA_DCR_DSIZE(0u) |
			DMA_DCR_D_REQ_MASK;
}

void __attribute__ ((interrupt)) DMA3_IRQHandler(void)
{
	DMA0->DMA[3u].DSR_BCR = DMA_DSR_BCR_DONE_MASK;
	//test end
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	NVIC_SetPriority(DMA0_IRQn, DMA_CHAN0_EXPT_PRI);
	NVIC_EnableIRQ(DMA0_IRQn);

	NVIC_SetPriority(DMA1_IRQn, DMA_CHAN1_EXPT_PRI);
	NVIC_EnableIRQ(DMA1_IRQn);

	NVIC_SetPriority(DMA2_IRQn, DMA_CHAN2_EXPT_PRI);
	NVIC_EnableIRQ(DMA2_IRQn);

	NVIC_SetPriority(DMA3_IRQn, DMA_CHAN3_EXPT_PRI);
	NVIC_EnableIRQ(DMA3_IRQn);

	DMA0->DMA[0u].SAR = (uint32_t) memcpy_src;
	DMA0->DMA[0u].DAR = (uint32_t) memcpy_dst_cont;
	DMA0->DMA[0u].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(uint32_t) * MEM_BLK_LEN);

	DMA0->DMA[1u].SAR = (uint32_t) memcpy_src;
	DMA0->DMA[1u].DAR = (uint32_t) memcpy_dst_req;
	DMA0->DMA[1u].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(uint32_t) * MEM_BLK_LEN);

	DMAMUX0->CHCFG[1u] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_ALWEN0);

	DMA0->DMA[2u].SAR = (uint32_t) &memset_src;
	DMA0->DMA[2u].DAR = (uint32_t) memset_dst_cont;
	DMA0->DMA[2u].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(uint32_t) * MEM_BLK_LEN);

	DMA0->DMA[3u].SAR = (uint32_t) &memset_src;
	DMA0->DMA[3u].DAR = (uint32_t) memset_dst_req;
	DMA0->DMA[3u].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(uint32_t) * MEM_BLK_LEN);

	DMAMUX0->CHCFG[3u] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_ALWEN1);

	//test start
	DMA0->DMA[0u].DCR = DMA_DCR_EINT_MASK | DMA_DCR_SINC_MASK |
			DMA_DCR_SSIZE(0u) | DMA_DCR_DINC_MASK | DMA_DCR_DSIZE(0u) |
			DMA_DCR_START_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
