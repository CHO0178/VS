/*
 * Name: iic_fram_meas.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "wdog.h"
#include "iic_bm.h"
#include "led.h"
#include "print.h"
#include "str.h"
#include "assert.h"

#define UART1_EXPT_PRI		2u
#define ADC_EXPT_PRI		2u
#define PORTA_EXPT_PRI		2u

#define FRAM_BUS_ADDR		IIC_ADDR_FRAM_DEV0
#define FRAM_DATA_ADDR		0x3355u

#define ADC_MEAS_PERIOD		240000ul

#define TX_MSG_LEN_LIM		32u

static char tx_msg[TX_MSG_LEN_LIM];
static volatile uint8_t tx_msg_len, tx_msg_ind, adc_val;

static uint8_t fram_rd(uint8_t bus_addr, uint16_t data_addr, uint8_t *data);
static uint8_t fram_wr(uint8_t bus_addr, uint16_t data_addr, uint8_t data);

static void adc_init(void);

static void uart_init(void);
static void uart_print(char *prefix, uint8_t val);

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

void __attribute__ ((interrupt)) ADC0_IRQHandler(void)
{
	uint8_t adc_val_temp;

	adc_val_temp = ADC0->R[0u];

	if (adc_val != adc_val_temp) {
		adc_val = adc_val_temp;
		led_bindisp(adc_val);
	}
}

void __attribute__ ((interrupt)) PORTA_IRQHandler(void)
{
	uint8_t led_val, fram_res;

	//fram read
	if (PORT_BTN3->ISFR & IOMASK_BTN3) {
		PORT_BTN3->ISFR = IOMASK_BTN3;

		fram_res = fram_rd(FRAM_BUS_ADDR, FRAM_DATA_ADDR, &led_val);

		if (fram_res) {
			uart_print("error", fram_res);
		} else {
			led_bindisp(led_val);
			uart_print("read", led_val);
		}
	}

	//fram write
	if (PORT_BTN4->ISFR & IOMASK_BTN4) {
		PORT_BTN4->ISFR = IOMASK_BTN4;

		fram_res = fram_wr(FRAM_BUS_ADDR, FRAM_DATA_ADDR, adc_val);

		if (fram_res) {
			uart_print("error", fram_res);
		} else {
			uart_print("write", adc_val);
		}
	}
}

static uint8_t fram_rd(uint8_t bus_addr, uint16_t data_addr, uint8_t *data)
{
	uint8_t err_id = 1u;

	iic_bus_start(I2C1);

	if (iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(bus_addr))) {
		iic_bus_stop(I2C1);
		return err_id;
	}

	err_id++;

	if (iic_bus_tx(I2C1, data_addr >> 8u)) {
		iic_bus_stop(I2C1);
		return err_id;
	}

	err_id++;

	if (iic_bus_tx(I2C1, data_addr)) {
		iic_bus_stop(I2C1);
		return err_id;
	}

	err_id++;

	iic_bus_repstart(I2C1);

	if (iic_bus_tx(I2C1, IIC_MAKE_ADDR_RD(bus_addr))) {
		iic_bus_stop(I2C1);
		return err_id;
	}

	*data = iic_bus_rx(I2C1, 1u);

	iic_bus_stop(I2C1);

	return 0u;
}

static uint8_t fram_wr(uint8_t bus_addr, uint16_t data_addr, uint8_t data)
{
	uint8_t err_id = 1u;

	iic_bus_start(I2C1);

	if (iic_bus_tx(I2C1, IIC_MAKE_ADDR_WR(bus_addr))) {
		iic_bus_stop(I2C1);
		return err_id;
	}

	err_id++;

	if (iic_bus_tx(I2C1, data_addr >> 8u)) {
		iic_bus_stop(I2C1);
		return err_id;
	}

	err_id++;

	if (iic_bus_tx(I2C1, data_addr)) {
		iic_bus_stop(I2C1);
		return err_id;
	}

	err_id++;

	if (iic_bus_tx(I2C1, data)) {
		iic_bus_stop(I2C1);
		return err_id;
	}

	iic_bus_stop(I2C1);

	return 0u;
}

static void adc_init(void)
{
	NVIC_SetPriority(ADC0_IRQn, ADC_EXPT_PRI);
	NVIC_EnableIRQ(ADC0_IRQn);

	ADC0->CFG1 = ADC_CFG1_ADIV(ADC_CFG1_ADIV_VAL_DIV4);
	ADC0->SC2 = ADC_SC2_ADTRG_MASK;
	ADC0->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(ADC_SC3_AVGS_VAL_32SAMP);
	ADC0->SC1[0u] = ADC_SC1_AIEN_MASK | ADC_CHAN_POT1;

	SIM->SOPT7 = SIM_SOPT7_ADC0ALTTRGEN_MASK |
				SIM_SOPT7_ADC0TRGSEL(SIM_SOPT7_ADTGSL_VAL_PIT0);

	PIT->MCR = PIT_MCR_FRZ_MASK;
	PIT->CHANNEL[0u].LDVAL = ADC_MEAS_PERIOD - 1u;
	PIT->CHANNEL[0u].TCTRL = PIT_TCTRL_TEN_MASK;
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

static void uart_print(char *prefix, uint8_t val)
{
	if (!(UART1->C2 & UART_C2_TIE_MASK)) {

		tx_msg_len = 0u;
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], prefix);
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], " ");
		tx_msg_len += print_uhex(&tx_msg[tx_msg_len], 2u, val);
		tx_msg_len += strcpy(&tx_msg[tx_msg_len], "\n");

		tx_msg_ind = 0u;
		UART1->C2 = UART_C2_TIE_MASK | UART_C2_RIE_MASK | UART_C2_TE_MASK |  UART_C2_RE_MASK;
	}
}

int main(void)
{
	wdog_init(WDOG_CONF_LPOCLK_1024_CYCLES);

	led_init();

	PORT_I2C1_SCL_SENS->PCR[IOIND_I2C1_SCL_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
	PORT_I2C1_SDA_SENS->PCR[IOIND_I2C1_SDA_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);

	assert(iic_recovery(GPIO_I2C1_SCL_SENS, GPIO_I2C1_SDA_SENS, IOMASK_I2C1_SCL_SENS, IOMASK_I2C1_SDA_SENS));

	PORT_I2C1_SCL_SENS->PCR[IOIND_I2C1_SCL_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT6);
	PORT_I2C1_SDA_SENS->PCR[IOIND_I2C1_SDA_SENS] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT6);

	iic_init(I2C1, IIC_ICR_VAL_400KHZ);

	adc_init();
	uart_init();

	NVIC_SetPriority(PORTA_IRQn, PORTA_EXPT_PRI);
	NVIC_EnableIRQ(PORTA_IRQn);

	PORT_BTN3->PCR[IOIND_BTN3] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO) |
			PORT_PCR_IRQC(PORT_PCR_IRQC_VAL_INT_EFALL);

	PORT_BTN4->PCR[IOIND_BTN4] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO) |
			PORT_PCR_IRQC(PORT_PCR_IRQC_VAL_INT_EFALL);

	while (1) {
		wdog_refresh();
	}

	return 0;
}
