/*
 * Name: dac_lut_dma_e5746.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "sin_lut.h"

#define DMA0_EXPT_PRI		2u

#define PIT0_MOD			3000ul

#define DAC0_DAT0_ADDR		0x4003F000ul

const uint16_t sin_lut[SIN_LUT_DATA_LEN] = {SIN_LUT_DATA};

void dma_set_common(void);

void __attribute__ ((interrupt)) DMA0_IRQHandler(void)
{
	DMA0->DMA[0u].DSR_BCR = DMA_DSR_BCR_DONE_MASK;
	dma_set_common();
}

void dma_set_common(void)
{
	DMA0->DMA[0u].SAR = (uint32_t) sin_lut;
	DMA0->DMA[0u].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(uint16_t) * SIN_LUT_DATA_LEN);
	DMA0->DMA[0u].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_SINC_MASK | DMA_DCR_SSIZE(2u) | DMA_DCR_DSIZE(2u) | DMA_DCR_D_REQ_MASK;
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	DAC0->C0 = DAC_C0_DACEN_MASK;

	NVIC_SetPriority(DMA0_IRQn, DMA0_EXPT_PRI);
	NVIC_EnableIRQ(DMA0_IRQn);

	DMA0->DMA[0u].DAR = DAC0_DAT0_ADDR;
	dma_set_common();

	DMAMUX0->CHCFG[0u] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_TRIG_MASK |
			DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_ALWEN0);

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0u].LDVAL = PIT0_MOD - 1u;
	PIT->CHANNEL[0u].TCTRL = PIT_TCTRL_TEN_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
