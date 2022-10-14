/*
 * Name: bme.h
 * Author: Martin Stankus
 *
 */

#ifndef _BME_H_
#define _BME_H_

#include <stdint.h>

#define BME_AND_OP					0x04000000ul
#define BME_OR_OP					0x08000000ul
#define BME_XOR_OP					0x0C000000ul
#define BME_BFI_OP					0x10000000ul
#define BME_LAC1_OP					0x08000000ul
#define BME_LAS1_OP					0x0C000000ul
#define BME_UBFX_OP					0x10000000ul

#define BME_BFI_POS_SHFT			23u
#define BME_BFI_WID_SHFT			19u
#define BME_LAC1_POS_SHFT			21u
#define BME_LAS1_POS_SHFT			21u
#define BME_UBFX_POS_SHFT			23u
#define BME_UBFX_WID_SHFT			19u

#define BME_AND(ADDR)				(BME_AND_OP | ((uint32_t) (ADDR)))
#define BME_OR(ADDR)				(BME_OR_OP | ((uint32_t) (ADDR)))
#define BME_XOR(ADDR)				(BME_XOR_OP | ((uint32_t) (ADDR)))

#define BME_LAC1(ADDR, POS)			(BME_LAC1_OP |\
									(((uint32_t) (POS)) << BME_LAC1_POS_SHFT) |\
									((uint32_t) (ADDR)))

#define BME_LAS1(ADDR, POS)			(BME_LAS1_OP |\
									(((uint32_t) (POS)) << BME_LAS1_POS_SHFT) |\
									((uint32_t) (ADDR)))

#define BME_BFI(ADDR, POS, WID)		(BME_BFI_OP |\
									(((uint32_t) (POS)) << BME_BFI_POS_SHFT) |\
									(((uint32_t) ((WID) - 1u)) << BME_BFI_WID_SHFT) |\
									((uint32_t) (ADDR)))

#define BME_UBFX(ADDR, POS, WID)	(BME_UBFX_OP |\
									(((uint32_t) (POS)) << BME_UBFX_POS_SHFT) |\
									(((uint32_t) ((WID) - 1u)) << BME_UBFX_WID_SHFT) |\
									((uint32_t) (ADDR)))

#define BME_AND_B(ADDR)				(*((volatile uint8_t *) BME_AND(ADDR)))
#define BME_AND_H(ADDR)				(*((volatile uint16_t *) BME_AND(ADDR)))
#define BME_AND_W(ADDR)				(*((volatile uint32_t *) BME_AND(ADDR)))

#define BME_OR_B(ADDR)				(*((volatile uint8_t *) BME_OR(ADDR)))
#define BME_OR_H(ADDR)				(*((volatile uint16_t *) BME_OR(ADDR)))
#define BME_OR_W(ADDR)				(*((volatile uint32_t *) BME_OR(ADDR)))

#define BME_XOR_B(ADDR)				(*((volatile uint8_t *) BME_XOR(ADDR)))
#define BME_XOR_H(ADDR)				(*((volatile uint16_t *) BME_XOR(ADDR)))
#define BME_XOR_W(ADDR)				(*((volatile uint32_t *) BME_XOR(ADDR)))

#define BME_BFI_B(ADDR, POS, WID)	(*((volatile uint8_t *) BME_BFI(ADDR, POS, WID)))
#define BME_BFI_H(ADDR, POS, WID)	(*((volatile uint16_t *) BME_BFI(ADDR, POS, WID)))
#define BME_BFI_W(ADDR, POS, WID)	(*((volatile uint32_t *) BME_BFI(ADDR, POS, WID)))

#define BME_LAC1_B(ADDR, POS)		(*((volatile uint8_t *) BME_LAC1(ADDR, POS)))
#define BME_LAC1_H(ADDR, POS)		(*((volatile uint16_t *) BME_LAC1(ADDR, POS)))
#define BME_LAC1_W(ADDR, POS)		(*((volatile uint32_t *) BME_LAC1(ADDR, POS)))

#define BME_LAS1_B(ADDR, POS)		(*((volatile uint8_t *) BME_LAS1(ADDR, POS)))
#define BME_LAS1_H(ADDR, POS)		(*((volatile uint16_t *) BME_LAS1(ADDR, POS)))
#define BME_LAS1_W(ADDR, POS)		(*((volatile uint32_t *) BME_LAS1(ADDR, POS)))

#define BME_UBFX_B(ADDR, POS, WID)	(*((volatile uint8_t *) BME_UBFX(ADDR, POS, WID)))
#define BME_UBFX_H(ADDR, POS, WID)	(*((volatile uint16_t *) BME_UBFX(ADDR, POS, WID)))
#define BME_UBFX_W(ADDR, POS, WID)	(*((volatile uint32_t *) BME_UBFX(ADDR, POS, WID)))

#endif /* _BME_H_ */
