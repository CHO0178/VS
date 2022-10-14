/*
 * Name: res_alloc_io_frdm.h
 * Author: Martin Stankus
 *
 */

#ifndef _RES_ALLOC_IO_FRDM_H_
#define _RES_ALLOC_IO_FRDM_H_

#include "MKL25Z4.h"

// uart0 ***********************************************************************

#define PORT_UART0_RX				PORTA
#define IOIND_UART0_RX				1u

#define PORT_UART0_TX				PORTA
#define IOIND_UART0_TX				2u

// rgb led *********************************************************************

#define PORT_LEDR					PORTB
#define GPIO_LEDR					GPIOB
#define FGPIO_LEDR					FGPIOB
#define IOIND_LEDR					18u
#define IOMASK_LEDR					(1ul << IOIND_LEDR)
#define TPM_LEDRG					TPM2
#define TPMCH_LEDR					0u

#define PORT_LEDG					PORTB
#define GPIO_LEDG					GPIOB
#define FGPIO_LEDG					FGPIOB
#define IOIND_LEDG					19u
#define IOMASK_LEDG					(1ul << IOIND_LEDG)
#define TPMCH_LEDG					1u

#define PORT_LEDB					PORTD
#define GPIO_LEDB					GPIOD
#define FGPIO_LEDB					FGPIOD
#define IOIND_LEDB					1u
#define IOMASK_LEDB					(1ul << IOIND_LEDB)
#define TPM_LEDB					TPM0
#define TPMCH_LEDB					1u

// i2c0 accelerometer **********************************************************

#define PORT_I2C0_SCL_ACC_FRDM		PORTE
#define GPIO_I2C0_SCL_ACC_FRDM		GPIOE
#define IOIND_I2C0_SCL_ACC_FRDM		24u
#define IOMASK_I2C0_SCL_ACC_FRDM	(1ul << IOIND_I2C0_SCL_ACC_FRDM)

#define PORT_I2C0_SDA_ACC_FRDM		PORTE
#define GPIO_I2C0_SDA_ACC_FRDM		GPIOE
#define IOIND_I2C0_SDA_ACC_FRDM		25u
#define IOMASK_I2C0_SDA_ACC_FRDM	(1ul << IOIND_I2C0_SDA_ACC_FRDM)

#define PORT_INT1_ACC_FRDM			PORTA
#define GPIO_INT1_ACC_FRDM			GPIOA
#define FGPIO_INT1_ACC_FRDM			FGPIOA
#define IOIND_INT1_ACC_FRDM			14u
#define IOMASK_INT1_ACC_FRDM		(1ul << IOIND_INT1_ACC_FRDM)

#define PORT_INT2_ACC_FRDM			PORTA
#define GPIO_INT2_ACC_FRDM			GPIOA
#define FGPIO_INT2_ACC_FRDM			FGPIOA
#define IOIND_INT2_ACC_FRDM			15u
#define IOMASK_INT2_ACC_FRDM		(1ul << IOIND_INT2_ACC_FRDM)

#endif /* _RES_ALLOC_IO_FRDM_H_ */
