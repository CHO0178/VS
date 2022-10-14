/*
 * Name: wdog.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "wdog.h"
#include "wdog_priv.h"

void wdog_init(WDOG_CONF wdog_conf)
{
	switch (wdog_conf) {
	case WDOG_CONF_DIS:

		SIM->COPC = 0ul;

		break;
	case WDOG_CONF_LPOCLK_32_CYCLES:

		SIM->COPC = SIM_COPC_COPT(SIM_COPC_COPT_SHORT);

		break;
	case WDOG_CONF_LPOCLK_256_CYCLES:

		SIM->COPC = SIM_COPC_COPT(SIM_COPC_COPT_MED);

		break;
	case WDOG_CONF_LPOCLK_1024_CYCLES:

		SIM->COPC = SIM_COPC_COPT(SIM_COPC_COPT_LONG);

		break;
	case WDOG_CONF_BUSCLK_8192_CYCLES:

		SIM->COPC = SIM_COPC_COPT(SIM_COPC_COPT_SHORT) | SIM_COPC_COPCLKS_MASK;

		break;
	case WDOG_CONF_BUSCLK_65536_CYCLES:

		SIM->COPC = SIM_COPC_COPT(SIM_COPC_COPT_MED) | SIM_COPC_COPCLKS_MASK;

		break;
	case WDOG_CONF_BUSCLK_262144_CYCLES:

		SIM->COPC = SIM_COPC_COPT(SIM_COPC_COPT_LONG) | SIM_COPC_COPCLKS_MASK;

		break;
	default:

		SIM->COPC = 0ul;

		break;
	}
}

void wdog_refresh(void)
{
	SIM->SRVCOP = WDOG_REFRESH_SEQ_A;
	SIM->SRVCOP = WDOG_REFRESH_SEQ_B;
}
