/*
 * Name: dma_singen.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "sin_lut.h"

#define DMA0_EXPT_PRI		2u
#define DMA1_EXPT_PRI		3u

#define TIMEBASE_MIN		100ul
#define TIMEBASE_MAX		3750ul

#define POT_MEAS_PERIOD		120000ul

#define DAC0_DAT0_ADDR		0x4003F000ul

const uint16_t sin_lut[SIN_LUT_DATA_LEN] = {SIN_LUT_DATA};

void dma_dac_set_common(void);
void dma_adc_set_common(void);

void __attribute__ ((interrupt)) DMA0_IRQHandler(void)
{
	DMA0->DMA[0u].DSR_BCR = DMA_DSR_BCR_DONE_MASK;
	dma_dac_set_common();
}

void __attribute__ ((interrupt)) DMA1_IRQHandler(void)
{
	DMA0->DMA[1u].DSR_BCR = DMA_DSR_BCR_DONE_MASK;
	dma_adc_set_common();
}

void dma_dac_set_common(void)
{
	DMA0->DMA[0u].SAR = (uint32_t) sin_lut;
	DMA0->DMA[0u].DSR_BCR = DMA_DSR_BCR_BCR(sizeof(uint16_t) * SIN_LUT_DATA_LEN);
	DMA0->DMA[0u].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_SINC_MASK | DMA_DCR_SSIZE(2u) | DMA_DCR_DSIZE(2u) | DMA_DCR_D_REQ_MASK;
}

void dma_adc_set_common(void)
{
	DMA0->DMA[1u].DSR_BCR = DMA_DSR_BCR_BCR(0x10000u);
	DMA0->DMA[1u].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
			DMA_DCR_D_REQ_MASK;
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	DAC0->C0 = DAC_C0_DACEN_MASK;

	NVIC_SetPriority(DMA0_IRQn, DMA0_EXPT_PRI);
	NVIC_EnableIRQ(DMA0_IRQn);

	DMA0->DMA[0u].DAR = DAC0_DAT0_ADDR;
	dma_dac_set_common();

	DMAMUX0->CHCFG[0u] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_TPM0OVF);

	TPM0->MOD = TIMEBASE_MAX - 1u;
	TPM0->SC = TPM_SC_DMA_MASK | TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | TPM_SC_PS(TPM_SC_PS_VAL_D2);

	NVIC_SetPriority(DMA1_IRQn, DMA1_EXPT_PRI);
	NVIC_EnableIRQ(DMA1_IRQn);

	DMA0->DMA[1u].SAR = (uint32_t) &ADC0->R[0];
	DMA0->DMA[1u].DAR = (uint32_t) &TPM0->MOD;
	dma_adc_set_common();

	DMAMUX0->CHCFG[1u] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_ADC0);

	ADC0->CFG1 = ADC_CFG1_ADIV(ADC_CFG1_ADIV_VAL_DIV4) | ADC_CFG1_MODE(ADC_CFG1_MODE_VAL_12BIT);
	ADC0->SC2 = ADC_SC2_ADTRG_MASK | ADC_SC2_ACFE_MASK | ADC_SC2_ACREN_MASK | ADC_SC2_DMAEN_MASK;
	ADC0->CV1 = TIMEBASE_MAX;
	ADC0->CV2 = TIMEBASE_MIN;
	ADC0->SC1[0u] = ADC_CHAN_POT1;

	SIM->SOPT7 = SIM_SOPT7_ADC0ALTTRGEN_MASK |
				SIM_SOPT7_ADC0TRGSEL(SIM_SOPT7_ADTGSL_VAL_PIT0);

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0u].LDVAL = POT_MEAS_PERIOD - 1u;
	PIT->CHANNEL[0u].TCTRL = PIT_TCTRL_TEN_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}
