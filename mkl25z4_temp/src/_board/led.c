/*
 * Name: led.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "led.h"
#include "led_priv.h"

#include "soc_def.h"
#include "res_alloc.h"

void led_init(void)
{
	uint8_t ind;

	LED_HIGH_GPIO->PSOR = LED_HIGH_MASK;
	LED_HIGH_GPIO->PDDR |= LED_HIGH_MASK;

	LED_LOW_GPIO->PSOR = LED_LOW_MASK;
	LED_LOW_GPIO->PDDR |= LED_LOW_MASK;

	for (ind = LED_HIGH_IND_BASE; ind < LED_HIGH_IND_BASE + LED_GRP_SIZE; ind++) {
		LED_HIGH_PORT->PCR[ind] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
	}

	for (ind = LED_LOW_IND_BASE; ind < LED_LOW_IND_BASE + LED_GRP_SIZE; ind++) {
		LED_LOW_PORT->PCR[ind] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_GPIO);
	}
}

void led_rgb_init(void)
{
	TPM_LEDRG->CONTROLS[TPMCH_LEDR].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	TPM_LEDRG->CONTROLS[TPMCH_LEDR].CnV = 0ul;

	TPM_LEDRG->CONTROLS[TPMCH_LEDG].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	TPM_LEDRG->CONTROLS[TPMCH_LEDG].CnV = 0ul;

	TPM_LEDRG->MOD = LED_RGB_PWM_PERIOD - 1u;
	TPM_LEDRG->CONF = TPM_CONF_DBGMODE(TPM_CONF_DBGM_VAL_RUN);
	TPM_LEDRG->SC = TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | TPM_SC_PS(TPM_SC_PS_VAL_D2);

	TPM_LEDB->CONTROLS[TPMCH_LEDB].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	TPM_LEDB->CONTROLS[TPMCH_LEDB].CnV = 0ul;

	TPM_LEDB->MOD = LED_RGB_PWM_PERIOD - 1u;
	TPM_LEDB->CONF = TPM_CONF_DBGMODE(TPM_CONF_DBGM_VAL_RUN);
	TPM_LEDB->SC = TPM_SC_CMOD(TPM_SC_CMOD_VAL_INTCLK) | TPM_SC_PS(TPM_SC_PS_VAL_D2);

	PORT_LEDR->PCR[IOIND_LEDR] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_LEDG->PCR[IOIND_LEDG] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT3);
	PORT_LEDB->PCR[IOIND_LEDB] = PORT_PCR_MUX(PORT_PCR_MUX_VAL_ALT4);
}

void led_high_bgraph(uint8_t len)
{
	LED_HIGH_GPIO->PSOR = (LED_HIGH_MASK << len) & LED_HIGH_MASK;
	LED_HIGH_GPIO->PCOR = (~(LED_HIGH_MASK << len)) & LED_HIGH_MASK;
}

void led_low_bgraph(uint8_t len)
{
	LED_LOW_GPIO->PSOR = (LED_LOW_MASK << len) & LED_LOW_MASK;
	LED_LOW_GPIO->PCOR = (~(LED_LOW_MASK << len)) & LED_LOW_MASK;
}

void led_bgraph(uint8_t len)
{
	led_high_bgraph((len > LED_GRP_SIZE) ? len - LED_GRP_SIZE : 0u);
	led_low_bgraph(len);
}

void led_high_bindisp(uint8_t val)
{
	LED_HIGH_GPIO->PSOR = ~(((uint32_t) val) << LED_HIGH_IND_BASE) & LED_HIGH_MASK;
	LED_HIGH_GPIO->PCOR = (((uint32_t) val) << LED_HIGH_IND_BASE) & LED_HIGH_MASK;
}

void led_low_bindisp(uint8_t val)
{
	LED_LOW_GPIO->PSOR = ~(((uint32_t) val) << LED_LOW_IND_BASE) & LED_LOW_MASK;
	LED_LOW_GPIO->PCOR = (((uint32_t) val) << LED_LOW_IND_BASE) & LED_LOW_MASK;
}

void led_bindisp(uint8_t val)
{
	led_high_bindisp(val >> LED_GRP_SIZE);
	led_low_bindisp(val);
}

void led_on(uint8_t id)
{
	LED_HIGH_GPIO->PCOR = (1ul << (LED_HIGH_IND_BASE + id - LED_GRP_SIZE - 1u)) & LED_HIGH_MASK;
	LED_LOW_GPIO->PCOR = (1ul << (LED_LOW_IND_BASE + id - 1u)) & LED_LOW_MASK;
}

void led_off(uint8_t id)
{
	LED_HIGH_GPIO->PSOR = (1ul << (LED_HIGH_IND_BASE + id - LED_GRP_SIZE - 1u)) & LED_HIGH_MASK;
	LED_LOW_GPIO->PSOR = (1ul << (LED_LOW_IND_BASE + id - 1u)) & LED_LOW_MASK;
}

void led_toggle(uint8_t id)
{
	LED_HIGH_GPIO->PTOR = (1ul << (LED_HIGH_IND_BASE + id - LED_GRP_SIZE - 1u)) & LED_HIGH_MASK;
	LED_LOW_GPIO->PTOR = (1ul << (LED_LOW_IND_BASE + id - 1u)) & LED_LOW_MASK;
}

void led_rgb_set(uint8_t id, uint32_t duty)
{
	if (id & LED_ID_RED) {
		TPM_LEDRG->CONTROLS[TPMCH_LEDR].CnV = duty;
	}

	if (id & LED_ID_GREEN) {
		TPM_LEDRG->CONTROLS[TPMCH_LEDG].CnV = duty;
	}

	if (id & LED_ID_BLUE) {
		TPM_LEDB->CONTROLS[TPMCH_LEDB].CnV = duty;
	}
}
