/*
 * Name: res_alloc_io_dock.h
 * Author: Martin Stankus
 *
 */

#ifndef _RES_ALLOC_IO_DOCK_H_
#define _RES_ALLOC_IO_DOCK_H_

#include "MKL25Z4.h"

// uart1 rs232 *****************************************************************

#define PORT_UART1_RX				PORTC
#define IOIND_UART1_RX				3u

#define PORT_UART1_TX				PORTC
#define IOIND_UART1_TX				4u

// uart2 ***********************************************************************

#define PORT_UART2_RX				PORTD
#define IOIND_UART2_RX				4u

#define PORT_UART2_TX				PORTD
#define IOIND_UART2_TX				5u

// spi1 accelerometer **********************************************************

#define PORT_ACC_DOCK_SPI1_MISO		PORTD
#define IOIND_ACC_DOCK_SPI1_MISO	7u

#define PORT_ACC_DOCK_SPI1_MOSI		PORTE
#define IOIND_ACC_DOCK_SPI1_MOSI	3u

#define PORT_ACC_DOCK_SPI1_SCK		PORTE
#define IOIND_ACC_DOCK_SPI1_SCK		2u

#define PORT_ACC_DOCK_SPI1_PCS0		PORTE
#define GPIO_ACC_DOCK_SPI1_PCS0		GPIOE
#define FGPIO_ACC_DOCK_SPI1_PCS0	FGPIOE
#define IOIND_ACC_DOCK_SPI1_PCS0	4u
#define IOMASK_ACC_DOCK_SPI1_PCS0	(1ul << IOIND_ACC_DOCK_SPI1_PCS0)

#define PORT_ACC_DOCK_INT1			PORTD
#define GPIO_ACC_DOCK_INT1			GPIOD
#define FGPIO_ACC_DOCK_INT1			FGPIOD
#define IOIND_ACC_DOCK_INT1			6u
#define IOMASK_ACC_DOCK_INT1		(1ul << IOIND_ACC_DOCK_INT1)

#define PORT_ACC_DOCK_RST			PORTE
#define GPIO_ACC_DOCK_RST			GPIOE
#define FGPIO_ACC_DOCK_RST			FGPIOE
#define IOIND_ACC_DOCK_RST			5u
#define IOMASK_ACC_DOCK_RST			(1ul << IOIND_ACC_DOCK_RST)

// i2c0 display ****************************************************************

#define PORT_I2C0_SCL_DISP			PORT_I2C0_SCL_DISP_ALT1
#define GPIO_I2C0_SCL_DISP			GPIO_I2C0_SCL_DISP_ALT1
#define IOIND_I2C0_SCL_DISP 		IOIND_I2C0_SCL_DISP_ALT1
#define IOMASK_I2C0_SCL_DISP		IOMASK_I2C0_SCL_DISP_ALT1

#define PORT_I2C0_SDA_DISP			PORT_I2C0_SDA_DISP_ALT1
#define GPIO_I2C0_SDA_DISP			GPIO_I2C0_SDA_DISP_ALT1
#define IOIND_I2C0_SDA_DISP 		IOIND_I2C0_SDA_DISP_ALT1
#define IOMASK_I2C0_SDA_DISP 		IOMASK_I2C0_SDA_DISP_ALT1

#define PORT_I2C0_SCL_DISP_ALT1		PORTC
#define GPIO_I2C0_SCL_DISP_ALT1		GPIOC
#define IOIND_I2C0_SCL_DISP_ALT1	8u
#define IOMASK_I2C0_SCL_DISP_ALT1	(1ul << IOIND_I2C0_SCL_DISP_ALT1)

#define PORT_I2C0_SDA_DISP_ALT1		PORTC
#define GPIO_I2C0_SDA_DISP_ALT1		GPIOC
#define IOIND_I2C0_SDA_DISP_ALT1	9u
#define IOMASK_I2C0_SDA_DISP_ALT1	(1ul << IOIND_I2C0_SDA_DISP_ALT1)

#define PORT_I2C0_SCL_DISP_ALT2		PORTB
#define GPIO_I2C0_SCL_DISP_ALT2		GPIOB
#define IOIND_I2C0_SCL_DISP_ALT2	0u
#define IOMASK_I2C0_SCL_DISP_ALT2	(1ul << IOIND_I2C0_SCL_DISP_ALT2)

#define PORT_I2C0_SDA_DISP_ALT2		PORTB
#define GPIO_I2C0_SDA_DISP_ALT2		GPIOB
#define IOIND_I2C0_SDA_DISP_ALT2	1u
#define IOMASK_I2C0_SDA_DISP_ALT2	(1ul << IOIND_I2C0_SDA_DISP_ALT2)

// i2c1 sensors and fram *******************************************************

#define PORT_I2C1_SCL_SENS			PORTE
#define GPIO_I2C1_SCL_SENS			GPIOE
#define IOIND_I2C1_SCL_SENS			1u
#define IOMASK_I2C1_SCL_SENS		(1ul << IOIND_I2C1_SCL_SENS)

#define PORT_I2C1_SDA_SENS			PORTE
#define GPIO_I2C1_SDA_SENS			GPIOE
#define IOIND_I2C1_SDA_SENS			0u
#define IOMASK_I2C1_SDA_SENS		(1ul << IOIND_I2C1_SDA_SENS)

#define PORT_INT_PULSE				PORTA
#define GPIO_INT_PULSE				GPIOA
#define FGPIO_INT_PULSE				FGPIOA
#define IOIND_INT_PULSE				16u
#define IOMASK_INT_PULSE			(1ul << IOIND_INT_PULSE)

#define PORT_INT_RANGE				PORTA
#define GPIO_INT_RANGE				GPIOA
#define FGPIO_INT_RANGE				FGPIOA
#define IOIND_INT_RANGE				17u
#define IOMASK_INT_RANGE			(1ul << IOIND_INT_FS)

#define PORT_XSDN_RANGE				PORTC
#define GPIO_XSDN_RANGE				GPIOC
#define FGPIO_XSDN_RANGE			FGPIOC
#define IOIND_XSDN_RANGE			17u
#define IOMASK_XSDN_RANGE			(1ul << IOIND_XSDN_FS)

// bnc connectors **************************************************************

#define PORT_J8						PORTE
#define GPIO_J8						GPIOE
#define FGPIO_J8					FGPIOE
#define IOIND_J8					20u
#define IOMASK_J8					(1ul << IOIND_J8)
#define TPM_J8						TPM1
#define TPMCH_J8					0u

#define PORT_J9						PORTE
#define GPIO_J9						GPIOE
#define FGPIO_J9					FGPIOE
#define IOIND_J9					21u
#define IOMASK_J9					(1ul << IOIND_J9)
#define TPM_J9						TPM1
#define TPMCH_J9					1u

#define PORT_J10					PORTE
#define GPIO_J10					GPIOE
#define FGPIO_J10					FGPIOE
#define IOIND_J10					22u
#define IOMASK_J10					(1ul << IOIND_J10)
#define TPM_J10						TPM2
#define TPMCH_J10					0u

#define PORT_J11					PORTE
#define GPIO_J11					GPIOE
#define FGPIO_J11					FGPIOE
#define IOIND_J11					23u
#define IOMASK_J11					(1ul << IOIND_J11)
#define TPM_J11						TPM2
#define TPMCH_J11					1u

#define PORT_J12					PORTE
#define GPIO_J12					GPIOE
#define FGPIO_J12					FGPIOE
#define IOIND_J12					29u
#define IOMASK_J12					(1ul << IOIND_J12)
#define TPM_J12						TPM0
#define TPMCH_J12					2u

#define PORT_J13					PORTE
#define GPIO_J13					GPIOE
#define FGPIO_J13					FGPIOE
#define IOIND_J13					30u
#define IOMASK_J13					(1ul << IOIND_J13)
#define TPM_J13						TPM0
#define TPMCH_J13					3u

#define PORT_J14					PORTC
#define GPIO_J14					GPIOC
#define FGPIO_J14					FGPIOC
#define IOIND_J14					0u
#define IOMASK_J14					(1ul << IOIND_J14)

#define PORT_J15					PORTC
#define GPIO_J15					GPIOC
#define FGPIO_J15					FGPIOC
#define IOIND_J15					5u
#define IOMASK_J15					(1ul << IOIND_J15)

#define PORT_J16					PORTC
#define GPIO_J16					GPIOC
#define FGPIO_J16					FGPIOC
#define IOIND_J16					6u
#define IOMASK_J16					(1ul << IOIND_J16)

#define PORT_J17					PORTC
#define GPIO_J17					GPIOC
#define FGPIO_J17					FGPIOC
#define IOIND_J17					7u
#define IOMASK_J17					(1ul << IOIND_J17)

// buttons *********************************************************************

#define PORT_BTN1					PORTA
#define GPIO_BTN1					GPIOA
#define FGPIO_BTN1					FGPIOA
#define IOIND_BTN1					4u
#define IOMASK_BTN1					(1ul << IOIND_BTN1)

#define PORT_BTN2					PORTA
#define GPIO_BTN2					GPIOA
#define FGPIO_BTN2					FGPIOA
#define IOIND_BTN2					5u
#define IOMASK_BTN2					(1ul << IOIND_BTN2)

#define PORT_BTN3					PORTA
#define GPIO_BTN3					GPIOA
#define FGPIO_BTN3					FGPIOA
#define IOIND_BTN3					12u
#define IOMASK_BTN3					(1ul << IOIND_BTN3)

#define PORT_BTN4					PORTA
#define GPIO_BTN4					GPIOA
#define FGPIO_BTN4					FGPIOA
#define IOIND_BTN4					13u
#define IOMASK_BTN4					(1ul << IOIND_BTN4)

// temp sensor *****************************************************************

#define PORT_TMP05					PORTE
#define IOIND_TMP05					31u
#define TPM_TMP05					TPM0
#define TPMCH_TMP05					4u

#endif /* _RES_ALLOC_IO_DOCK_H_ */
