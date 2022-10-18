#include "wdog.h"
#include "littleHelper.h"
#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "led.h"
#include "sin_lut.h"

// compare trig
/*
DAC
	DAT0L
		DATA0			lower data pro pøevod
	DAT0H
		DATA1			higher data pro pøevod
	C0
		DACEN			povolí použití 12bit DAC
CMP
	DACCR
		DACEN			povolí napájení vnitøního DAC
		VOSEL			voltage select
	MUXCR
		PSEL			multiplex to positive input of CMP
		MSEL			multiplex to negative input of CMP
	SRC
		IER				enable flag raising
		CFR				flag interruptu
	CR1
		OPE				enable output
		EN				enable power for CMP
*/

#define PIT_EXPT_PRI		2u
#define CMP_EXPT_PRI		2u
#define PIT0_MOD			3000ul
#define DAC0_DAT0			(*((volatile uint16_t *) 0x4003F000ul))
#define CMP_EXPT_CNT_MOD	50u


uint16_t sin_lut_ind, cmp_expt_cnt, disp_cnt;
const uint16_t sin_lut[SIN_LUT_DATA_LEN] = {SIN_LUT_DATA};

void setupNVICandPIT();


int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);
	led_init();
	setupNVICandPIT();

	// povolte použití 12bit DAC
	DAC0->C0 = DAC_C0_DACEN_MASK;

	// pøepnìte multiplex v periferii port tak, aby pøesmìroval signál z BNC konektoru J15 na výstup periferie CMP0
	PORT_J15->PCR[IOIND_J15] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT6);
	// pøepnìte multiplex v periferii port tak, aby pøesmìroval signál z výstupu DAC na BNC konektor
	// nastavte priferii CMP tak aby výstupní hodnota vosel byla 0x20u
	CMP0->DACCR = CMP_DACCR_DACEN_MASK | CMP_DACCR_VOSEL(0x20u);
	// nastavte vstupní multiplexory na kladný vstup periferii DAC0 a záporný vstup signal DAC periferie CMP
	CMP0->MUXCR = CMP_MUXCR_PSEL(4u) | CMP_MUXCR_MSEL(7u);
	// povol flag raising v periferii CMP
	CMP0->SCR = CMP_SCR_IER_MASK;
	// povol výstup a napájení periferie CMP
	CMP0->CR1 = CMP_CR1_OPE_MASK | CMP_CR1_EN_MASK;

	while (1) {
		wdog_refresh();
	}

	return 0;
}


void setupNVICandPIT()
{
	NVIC_SetPriority(PIT_IRQn, PIT_EXPT_PRI);
	NVIC_EnableIRQ(PIT_IRQn);

	NVIC_SetPriority(CMP0_IRQn, PIT_EXPT_PRI);
	NVIC_EnableIRQ(CMP0_IRQn);

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0u].LDVAL = PIT0_MOD - 1u;
	PIT->CHANNEL[0u].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

}


void __attribute__ ((interrupt)) PIT_IRQHandler(void)
{
	PIT->CHANNEL[0u].TFLG = PIT_TFLG_TIF_MASK;

	DAC0_DAT0 = sin_lut[sin_lut_ind];

	sin_lut_ind++;
	if (sin_lut_ind == SIN_LUT_DATA_LEN) {
		sin_lut_ind = 0;
	}
}

void __attribute__ ((interrupt)) CMP0_IRQHandler(void)
{
	// proveïte vynulování vyhozeného flagu (dejte si pozor aby jste nevynulovali povolení flagù)
	CMP0->SCR = CMP_SCR_IER_MASK | CMP_SCR_CFR_MASK;

	// èítej poèet vyvolaných interruptù a každé 50 volání zvìtši výstup na diodì o 1
	cmp_expt_cnt++;
	if (cmp_expt_cnt == CMP_EXPT_CNT_MOD) {
		cmp_expt_cnt = 0;
		led_bindisp(++disp_cnt);
	}
}
