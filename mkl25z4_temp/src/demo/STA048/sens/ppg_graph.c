/*
 * Name: ppg_graph.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "led.h"
#include "expt.h"
#include "wdog.h"
#include "assert.h"
#include "pulse.h"

#include "iic_bm.h"
#include "disp_bm.h"
#include "vcom_bm.h"
#include "wait_bm.h"

#define PWM_RES_BITS					12u
#define PWM_PERIOD						(1ul << PWM_RES_BITS)

#define ADC_TRIG_TPM_SC_PS_VAL			TPM_SC_PS_VAL_D64
#define ADC_TRIG_MEAS_PERIOD			37500ul
#define ADC_TRIG_MEAS_A_OFFSET			9375ul
#define ADC_TRIG_MEAS_B_OFFSET			18750ul

volatile uint8_t pulse_rge_val = PULSE_SPO2_CONF_RGE2048NA;
volatile uint16_t pulse_led1_pa_val = 0u;
volatile uint16_t pulse_led2_pa_val = 0u;

static void adc_handler(void);
static void ext_int_handler(void);

static void pulse_init(void);
static void pot_init(void);
static void pwm_init(void);
static void btn_init(void);
static void ext_int_init(void);

static void process_int_btn(void);
static void process_int_pulse(void);

static void adc_handler(void)
{
	pulse_led1_pa_val = ADC0->R[0u];
	pulse_led2_pa_val = ADC0->R[1u];
}

static void ext_int_handler(void)
{
	if (PORT_BTN1->ISFR & IOMASK_BTN1) {
		PORT_BTN1->ISFR = IOMASK_BTN1;
		process_int_btn();

	}

	if (PORT_INT_PULSE->ISFR & IOMASK_INT_PULSE) {
		PORT_INT_PULSE->ISFR = IOMASK_INT_PULSE;
		process_int_pulse();
	}
}

static void pulse_init(void)
{
	PORT_I2C1_SCL_SENS->PCR[IOIND_I2C1_SCL_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
	PORT_I2C1_SDA_SENS->PCR[IOIND_I2C1_SDA_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	assert(iic_recovery(GPIO_I2C1_SCL_SENS, GPIO_I2C1_SDA_SENS, IOMASK_I2C1_SCL_SENS, IOMASK_I2C1_SDA_SENS));

	PORT_I2C1_SCL_SENS->PCR[IOIND_I2C1_SCL_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT6);
	PORT_I2C1_SDA_SENS->PCR[IOIND_I2C1_SDA_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT6);

	iic_init(I2C1, IIC_ICR_VAL_400KHZ);
}

static void pot_init(void)
{
	expt_vect_set(ADC0_IRQn, adc_handler);
	NVIC_SetPriority(ADC0_IRQn, 0u);
	NVIC_EnableIRQ(ADC0_IRQn);

	ADC0->CFG1 = ADC_CFG1_ADIV(ADC_CFG1_ADIV_VAL_DIV4);
	ADC0->SC2 = ADC_SC2_ADTRG_MASK;
	ADC0->SC1[0u] = ADC_CHAN_POT1;
	ADC0->SC1[1u] = ADC_SC1_AIEN_MASK | ADC_CHAN_POT2;

	TPM1->CONTROLS[0u].CnSC = TPM_CnSC_MSA_MASK;
	TPM1->CONTROLS[0u].CnV = ADC_TRIG_MEAS_A_OFFSET;

	TPM1->CONTROLS[1u].CnSC = TPM_CnSC_MSA_MASK;
	TPM1->CONTROLS[1u].CnV = ADC_TRIG_MEAS_B_OFFSET;

	TPM1->MOD = ADC_TRIG_MEAS_PERIOD - 1u;
	TPM1->SC = TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | TPM_SC_PS(ADC_TRIG_TPM_SC_PS_VAL);
}

static void pwm_init(void)
{
	PORT_J12->PCR[IOIND_J12] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_J13->PCR[IOIND_J13] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);

	//J12
	TPM0->CONTROLS[2u].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;

	//J13
	TPM0->CONTROLS[3u].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;

	TPM0->MOD = 4094u;
	TPM0->CONF = TPM_CONF_DBGMODE(TPM_CONF_DBGM_VAL_RUN);
	TPM0->SC = TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | TPM_SC_PS(TPM_SC_PS_VAL_D1);
}

static void btn_init(void)
{
	led_toggle(LED_ID1);
	PORT_BTN1->PCR[IOIND_BTN1] = PORT_PCR_IRQC(PORT_PCR_IRQC_VAL_INT_EFALL) |
			PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
}

static void ext_int_init(void)
{
	expt_vect_set(PORTA_IRQn, ext_int_handler);
	expt_vect_set(PORTD_IRQn, ext_int_handler);

	NVIC_SetPriority(PORTA_IRQn, 0u);
	NVIC_SetPriority(PORTD_IRQn, 0u);

	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTD_IRQn);
}

static void process_int_btn(void)
{
	switch (pulse_rge_val) {
	case PULSE_SPO2_CONF_RGE2048NA:

		led_toggle(LED_ID1);
		led_toggle(LED_ID2);
		pulse_rge_val = PULSE_SPO2_CONF_RGE4096NA;

		break;
	case PULSE_SPO2_CONF_RGE4096NA:

		led_toggle(LED_ID2);
		led_toggle(LED_ID3);
		pulse_rge_val = PULSE_SPO2_CONF_RGE8192NA;

		break;
	case PULSE_SPO2_CONF_RGE8192NA:

		led_toggle(LED_ID3);
		led_toggle(LED_ID4);
		pulse_rge_val = PULSE_SPO2_CONF_RGE16384NA;

		break;
	case PULSE_SPO2_CONF_RGE16384NA:

		led_toggle(LED_ID4);
		led_toggle(LED_ID1);
		pulse_rge_val = PULSE_SPO2_CONF_RGE2048NA;

		break;
	default:
		assert(0);
		break;
	}
}

static void process_int_pulse(void)
{
	static uint8_t pulse_rge_val_old = 0xFFu;
	static uint16_t pulse_led1_pa_val_old = 0xFFFFu;
	static uint16_t pulse_led2_pa_val_old = 0xFFFFu;

	uint32_t red, ired;

	iic_bus_start(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(IIC_ADDR_PULSE)));
	assert(!iic_bus_tx(I2C1, PULSE_ADDR_FIFO_DATA));
	iic_bus_repstart(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_RD(IIC_ADDR_PULSE)));

	red = iic_bus_rx(I2C1, 0u);
	red <<= 8u;
	red |= iic_bus_rx(I2C1, 0u);
	red <<= 8u;
	red |= iic_bus_rx(I2C1, 0u);

	ired = iic_bus_rx(I2C1, 0u);
	ired <<= 8u;
	ired |= iic_bus_rx(I2C1, 0u);
	ired <<= 8u;
	ired |= iic_bus_rx(I2C1, 1u);

	iic_bus_stop(I2C1);

	TPM0->CONTROLS[2u].CnV = red >> 6;
	TPM0->CONTROLS[3u].CnV = ired >> 6;
}

int main(void)
{
	//volatile uint32_t val;

	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	ext_int_init();

	led_init();
	btn_init();
	pot_init();
	pwm_init();

	disp_init();
	pulse_init();

	//reset
	iic_bus_start(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(IIC_ADDR_PULSE)));
	assert(!iic_bus_tx(I2C1, PULSE_ADDR_MODE_CONF));
	assert(!iic_bus_tx(I2C1, PULSE_MODE_CONF_RESET));
	iic_bus_stop(I2C1);

	//clear interrupts
	iic_bus_start(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(IIC_ADDR_PULSE)));
	assert(!iic_bus_tx(I2C1, PULSE_ADDR_INT1STAT));
	iic_bus_repstart(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_RD(IIC_ADDR_PULSE)));
	iic_bus_rx(I2C1, 1u);
	iic_bus_stop(I2C1);

	iic_bus_start(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(IIC_ADDR_PULSE)));
	assert(!iic_bus_tx(I2C1, PULSE_ADDR_INT2STAT));
	iic_bus_repstart(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_RD(IIC_ADDR_PULSE)));
	iic_bus_rx(I2C1, 1u);
	iic_bus_stop(I2C1);

	iic_bus_start(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(IIC_ADDR_PULSE)));
	assert(!iic_bus_tx(I2C1, PULSE_ADDR_INT1EN));
	assert(!iic_bus_tx(I2C1,  PULSE_INT1_PPG_RDY));
	iic_bus_stop(I2C1);

	iic_bus_start(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(IIC_ADDR_PULSE)));
	assert(!iic_bus_tx(I2C1, PULSE_ADDR_LED1_PA));
	assert(!iic_bus_tx(I2C1, 30));
	iic_bus_stop(I2C1);

	iic_bus_start(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(IIC_ADDR_PULSE)));
	assert(!iic_bus_tx(I2C1, PULSE_ADDR_LED2_PA));
	assert(!iic_bus_tx(I2C1, 0x00));
	iic_bus_stop(I2C1);

	iic_bus_start(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(IIC_ADDR_PULSE)));
	assert(!iic_bus_tx(I2C1, PULSE_ADDR_SPO2_CONF));
	assert(!iic_bus_tx(I2C1, PULSE_SPO2_CONF_RGE4096NA | PULSE_SPO2_CONF_SR50HZ | PULSE_SPO2_CONF_RES15B));
	iic_bus_stop(I2C1);

	iic_bus_start(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(IIC_ADDR_PULSE)));
	assert(!iic_bus_tx(I2C1, PULSE_ADDR_INT1EN));
	assert(!iic_bus_tx(I2C1, PULSE_INT1_PPG_RDY));
	iic_bus_stop(I2C1);

	PORT_INT_PULSE->PCR[IOIND_INT_PULSE] = PORT_PCR_IRQC(PORT_PCR_IRQC_VAL_INT_EFALL) |
			PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

	iic_bus_start(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(IIC_ADDR_PULSE)));
	assert(!iic_bus_tx(I2C1, PULSE_ADDR_MODE_CONF));
	assert(!iic_bus_tx(I2C1, PULSE_MODE_CONF_MODESPO2));
	iic_bus_stop(I2C1);

	/*
	wait_bm(WAIT_BM_100MSEC_FSYS48M, 1u);
	wait_bm(WAIT_BM_100MSEC_FSYS48M, 1u);

	iic_bus_start(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(IIC_ADDR_PULSE)));
	assert(!iic_bus_tx(I2C1, PULSE_ADDR_FIFO_DATA));
	iic_bus_repstart(I2C1);
	assert(!iic_bus_tx(I2C1, IIC_MAKE_ADDR_RD(IIC_ADDR_PULSE)));
	iic_bus_rx(I2C1, 1u);
	iic_bus_rx(I2C1, 1u);
	iic_bus_rx(I2C1, 1u);
	iic_bus_rx(I2C1, 1u);
	iic_bus_rx(I2C1, 1u);
	iic_bus_rx(I2C1, 1u);
	iic_bus_stop(I2C1);

	val = GPIO_INT_PULSE->PDIR & IOMASK_INT_PULSE;
	*/

	while (1) {
		wdog_refresh();
	}

	return 0;
}
