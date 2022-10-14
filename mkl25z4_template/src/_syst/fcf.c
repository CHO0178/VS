/*
 * Name: fcf.c
 * Author: Martin Stankus
 *
 * Do not modify this file.
 * Wrong setting can permanently corrupt the MCU.
 *
 */

#include <stdint.h>

#include "fcf.h"

const uint8_t fcf[FCF_LEN] __attribute__ ((section (".fcf"))) __attribute__ ((used)) = {
	FCF_BACKKEY3, FCF_BACKKEY2, FCF_BACKKEY1, FCF_BACKKEY0,
	FCF_BACKKEY7, FCF_BACKKEY6, FCF_BACKKEY5, FCF_BACKKEY4,
	FCF_FPROT3, FCF_FPROT2, FCF_FPROT1, FCF_FPROT0,
	FCF_FSEC, FCF_FOPT, FCF_RSVD, FCF_RSVD
};
