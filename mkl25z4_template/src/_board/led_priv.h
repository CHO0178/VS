/*
 * Name: led_priv.h
 * Author: Martin Stankus
 *
 */

#ifndef _LED_PRIV_H_
#define _LED_PRIV_H_

#include "MKL25Z4.h"

#define LED_GRP_SIZE				4u

#define LED_HIGH_PORT				PORTC
#define LED_LOW_PORT				PORTB

#define LED_HIGH_GPIO				FGPIOC
#define LED_LOW_GPIO				FGPIOB

#define LED_HIGH_IND_BASE			10u
#define LED_LOW_IND_BASE			8u

#define LED_HIGH_MASK				(0xFul << LED_HIGH_IND_BASE)
#define LED_LOW_MASK				(0xFul << LED_LOW_IND_BASE)

#endif /* _LED_PRIV_H_ */
