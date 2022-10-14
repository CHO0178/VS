/*
 * Name: fcf.h
 * Author: Martin Stankus
 *
 * Do not modify this file.
 * Wrong setting can permanently corrupt the MCU.
 *
 */

#ifndef _FCF_H_
#define _FCF_H_

#ifndef SECURITY_EN
#define SECURITY_EN				0
#endif

#ifndef RST_PIN_EN
#define RST_PIN_EN				1
#endif

#ifndef NMI_EN
#define NMI_EN					0
#endif

#ifndef BACKKEY7
#define FCF_BACKKEY7			0xABu
#else
#define FCF_BACKKEY7			BACKKEY7
#endif

#ifndef BACKKEY6
#define FCF_BACKKEY6			0xBAu
#else
#define FCF_BACKKEY6			BACKKEY6
#endif

#ifndef BACKKEY5
#define FCF_BACKKEY5			0xFEu
#else
#define FCF_BACKKEY5			BACKKEY5
#endif

#ifndef BACKKEY4
#define FCF_BACKKEY4			0xEDu
#else
#define FCF_BACKKEY4			BACKKEY4
#endif

#ifndef BACKKEY3
#define FCF_BACKKEY3			0xC0u
#else
#define FCF_BACKKEY3			BACKKEY3
#endif

#ifndef BACKKEY2
#define FCF_BACKKEY2			0xDEu
#else
#define FCF_BACKKEY2			BACKKEY2
#endif

#ifndef BACKKEY1
#define FCF_BACKKEY1			0xFAu
#else
#define FCF_BACKKEY1			BACKKEY1
#endif

#ifndef BACKKEY0
#define FCF_BACKKEY0			0xCEu
#else
#define FCF_BACKKEY0			BACKKEY0
#endif

#define FCF_LEN					16u

#define FCF_FPROT_WR_PROT_OFF	0xFFu

#define FCF_FPROT3				FCF_FPROT_WR_PROT_OFF
#define FCF_FPROT2				FCF_FPROT_WR_PROT_OFF
#define FCF_FPROT1				FCF_FPROT_WR_PROT_OFF
#define FCF_FPROT0				FCF_FPROT_WR_PROT_OFF

#define FCF_FSEC_COMMON			0xBCu

#if SECURITY_EN
#define FCF_FSEC_SEC			0x03u
#else
#define FCF_FSEC_SEC			0x02u
#endif

#define FCF_FSEC				(FCF_FSEC_COMMON | FCF_FSEC_SEC)

#define FCF_FOPT_COMMON			0xF3u

#if RST_PIN_EN
#define FCF_FOPT_RST_PIN		0x08u
#else
#define FCF_FOPT_RST_PIN		0x00u
#endif

#if NMI_EN
#define FCF_FOPT_NMI			0x04u
#else
#define FCF_FOPT_NMI			0x00u
#endif

#define FCF_FOPT				(FCF_FOPT_COMMON | FCF_FOPT_RST_PIN | FCF_FOPT_NMI)

#define FCF_RSVD				0xFFu

#endif /* _FCF_H_ */
