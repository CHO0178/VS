/*
 * Name: test_example_blind.c
 * Author: Martin Stankus
 *
 * description:
 * 1. uart0 isr listens for characters '1' and '0' to switch led1 on/off
 * 2. btn1 selects whether red, green or blue led pwm is set by pot1
 * 3. adc samples pot1 with f = 25 Hz
 * 4. adc value is used as tpm channel value for respective led
 *
 * total points: 20
 * required minimal score: 5
 * all answers have to be defended
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"

#define PORTA_EXPT_PRI		2u
#define ADC_EXPT_PRI		2u
#define UART0_EXPT_PRI		3u

//TASK 1
//score: 2 points
//set sbr value for rate of 115200 Bd (module clock = 48 MHz)
//refman page: 762
#define UART_SBR_VAL		0u

//TASK 2
//score: 2 points
//set timer period so that timer frequency is 25 Hz (module clock = 24 MHz)
//refman page: 581
#define POT_MEAS_PERIOD		0ul

typedef enum {
	LED_SEL_RED,
	LED_SEL_GREEN,
	LED_SEL_BLUE,
} LED_SEL;

LED_SEL led_sel = LED_SEL_RED;

void __attribute__ ((interrupt)) PORTA_IRQHandler(void)
{
	//TASK 3
	//score: 2 points
	//clear btn1 interrupt flag
	//refman page: 186

	switch (led_sel) {
	case LED_SEL_RED:
		led_sel = LED_SEL_GREEN;

		//TASK 4, part 1
		//score: 1 points
		//turn led6 off
		//refman page: 776

		GPIOC->PCOR = 1ul << 12u;
		break;
	case LED_SEL_GREEN:
		led_sel = LED_SEL_BLUE;
		GPIOC->PSOR = 1ul << 12u;
		GPIOC->PCOR = 1ul << 13u;
		break;
	case LED_SEL_BLUE:
		led_sel = LED_SEL_RED;
		GPIOC->PSOR = 1ul << 13u;

		//TASK 4, part 2
		//score: 1 points
		//turn led6 on
		//refman page: 776

		break;
	default:
		break;
	}
}

void __attribute__ ((interrupt)) UART0_IRQHandler(void)
{
	UART0->S1;

	switch (UART0->D) {
	case '1':
		GPIOB->PCOR = 1ul << 8u;
		break;
	case '0':
		GPIOB->PSOR = 1ul << 8u;
		break;
	default:
		break;
	}
}

void __attribute__ ((interrupt)) ADC0_IRQHandler(void)
{
	switch (led_sel) {
	case LED_SEL_RED:
		TPM2->CONTROLS[0u].CnV = ADC0->R[0u];
		break;
	case LED_SEL_GREEN:
		TPM2->CONTROLS[1u].CnV = ADC0->R[0u];
		break;
	case LED_SEL_BLUE:
		TPM0->CONTROLS[1u].CnV = ADC0->R[0u];
		break;
	default:
		ADC0->R[0u];
		break;
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	GPIOC->PCOR = 1ul << 11;
	GPIOC->PDDR |= 1ul << 11;
	PORTC->PCR[11u] = PORT_PCR_MUX(1u);

	GPIOC->PSOR = 1ul << 12u;
	GPIOC->PDDR |= 1ul << 12u;
	PORTC->PCR[12u] = PORT_PCR_MUX(1u);

	GPIOC->PSOR = 1ul << 13u;
	GPIOC->PDDR |= 1ul << 13u;
	PORTC->PCR[13u] = PORT_PCR_MUX(1u);

	GPIOB->PSOR = 1ul << 8u;
	GPIOB->PDDR |= 1ul << 8u;
	PORTB->PCR[8u] = PORT_PCR_MUX(1u);

	NVIC_SetPriority(PORTA_IRQn, PORTA_EXPT_PRI);
	NVIC_EnableIRQ(PORTA_IRQn);

	PORTA->PCR[4u] = PORT_PCR_IRQC(10u) | PORT_PCR_MUX(1u);

	PORTB->PCR[18u] = PORT_PCR_MUX(3u);
	PORTB->PCR[19u] = PORT_PCR_MUX(3u);
	PORTD->PCR[1u] = PORT_PCR_MUX(4u);

	TPM2->CONTROLS[0u].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	TPM2->CONTROLS[1u].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;

	//TASK 5, part 1
	//score: 1 points
	//set internal clock mode (=0b01) and prescaler div by 8 for tpm2
	//refman page: 552

	TPM0->CONTROLS[1u].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;

	//TASK 5, part 2
	//score: 1 points
	//perform the same operation for tpm0
	//refman page: 552

	NVIC_SetPriority(ADC0_IRQn, ADC_EXPT_PRI);
	NVIC_EnableIRQ(ADC0_IRQn);

	//TASK 6
	//score: 2 points
	//set 16bit quantisation and clock divide by 4 for adc0
	//refman page: 465

	ADC0->SC2 = ADC_SC2_ADTRG_MASK;

	//TASK 7
	//score: 2 points
	//set interrupt enable and pot1 as input for adc0
	//refman page: 463

	SIM->SOPT7 = SIM_SOPT7_ADC0ALTTRGEN_MASK | SIM_SOPT7_ADC0TRGSEL(5u);

	PIT->MCR = PIT_MCR_FRZ_MASK;

	//TASK 8
	//score: 2 points
	//set load value of pit1 to POT_MEAS_PERIOD
	//refman page: 578

	PIT->CHANNEL[1u].TCTRL = PIT_TCTRL_TEN_MASK;

	PORTA->PCR[1u] = PORT_PCR_MUX(2u);

	NVIC_SetPriority(UART0_IRQn, UART0_EXPT_PRI);
	NVIC_EnableIRQ(UART0_IRQn);

	UART0->BDH = UART_BDH_SBR(UART_SBR_VAL >> 8u);
	UART0->BDL = UART_BDL_SBR(UART_SBR_VAL);

	//TASK 9
	//score: 2 points
	//enable receiver and receiver interrupt for uart0
	//refman page: 753

	while (1) {
		wdog_refresh();
	}

	return 0;
}

//everything works: 2 points
