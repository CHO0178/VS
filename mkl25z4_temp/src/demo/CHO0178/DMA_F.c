#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "led.h"

#define ADC_EXPT_PRI		2u

#define PIT0_MOD			2400ul

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();
	// DAC - chapter 30
	// povolte periferii DAC (C0)(DACEN = 1)
	DAC0->C0 = DAC_C0_DACEN_MASK;

	// povolte pending IRQ z DMA v NVICu na prioritu 2
	NVIC_SetPriority(DMA0_IRQn, ADC_EXPT_PRI);
	NVIC_EnableIRQ(DMA0_IRQn);

	// ADC - chapter 28
	// v analog to digital prevodniku nastavte divideRatio (ADIV = 2) na inputClock/4 a 12-bitovou konverzi(mode = 1)//(CFG1)
	ADC0->CFG1 = ADC_CFG1_ADIV(ADC_CFG1_ADIV_VAL_DIV4) | ADC_CFG1_MODE(1);
	// nastavte v ADC zvolte conversion triger na Hardware trigger(ADTRG = 1) a povolte generování DMA requestu (DMAEN = 1)//(SC2)
	ADC0->SC2 = ADC_SC2_ADTRG_MASK | ADC_SC2_DMAEN_MASK;
	// povolte generovani interuptu(AIEN = 1) a zvolte vstup na BNC konektor J9 (ADCH = 4)//(SC1)
	ADC0->SC1[0u] = ADC_SC1_AIEN_MASK | ADC_CHAN_J9;

	// DMA - chapter 23
	// nastavte zdrojovou adresu na registr R periferie ADC
	DMA0->DMA[0u].SAR = &ADC0->R[0u];
	// nastavte destination adress register na adresu registru DAC0_DAT0L
	DMA0->DMA[0u].DAR = 0x4003F000;
	// nastavte pocet opakovani pred vyvoanim preruseni na 500 (BCR = 500)
	DMA0->DMA[0u].DSR_BCR = DMA_DSR_BCR_BCR(500);
	// povolte generovani interuptu pri dokonceni prenosu(EINT = 1), peirferii vyzadat prenos(ERQ = 1), nastavte mod prenosu na cycle steal(CS = 1) a velikosti prenasenych hodnot na 2B (SSIZE = 2;DSIZE = 2)
	DMA0->DMA[0u].DCR = DMA_DCR_EINT_MASK | DMA_DCR_ERQ_MASK | DMA_DCR_CS_MASK |
						DMA_DCR_SSIZE(2u) | DMA_DCR_DSIZE(2u);
	// DMAMUX chapter 22
	// povolte kanal a nastavte zdroj ADC0 (str. 65)(ENBL)//(CHCFG)
	DMAMUX0->CHCFG[0u] = DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(40);

	// SIM chapter 12
	// povolte alternativni trigrovani modulu ADC0(ADC0ALTTRGEN = 1) a nastavte zdroj trigrovani na PIT0 (SOPT7)
	SIM->SOPT7 = SIM_SOPT7_ADC0ALTTRGEN_MASK |
				SIM_SOPT7_ADC0TRGSEL(SIM_SOPT7_ADTGSL_VAL_PIT0);

	// PIT chapter 32
	// povolte zastavení casovace behem debugingu
	PIT->MCR = PIT_MCR_FRZ_MASK;
	// nastavte obnovovaci frekvenci na 100us  CLK = 24MHz MODULO -> LDVAL
	PIT->CHANNEL[0u].LDVAL = PIT0_MOD - 1u;
	// povolte timer (TEN = 1)//(TCTRL)
	PIT->CHANNEL[0u].TCTRL = PIT_TCTRL_TEN_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}

// vytvorte handler z preruseni periferie DMA ve kterem nastavite hodnotu BCR na 500

void DMA0_IRQHandler(void)
{
	DMA0->DMA[0u].DSR_BCR = DMA_DSR_BCR_DONE_MASK;
	DMA0->DMA[0u].DSR_BCR = DMA_DSR_BCR_BCR(500);
}
