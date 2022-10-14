/*
 * Name: res_alloc_iic.h
 * Author: Martin Stankus
 *
 */

#ifndef _RES_ALLOC_IIC_H_
#define _RES_ALLOC_IIC_H_

#define IIC_ADDR_ACC_FRDM		0x1Du

#define IIC_ADDR_DISP			0x3Cu

#define IIC_ADDR_FRAM_DEV0		0x50u
#define IIC_ADDR_FRAM_DEV1		0x51u
#define IIC_ADDR_FRAM_DEV2		0x52u
#define IIC_ADDR_FRAM_DEV3		0x53u
#define IIC_ADDR_FRAM_DEV4		0x54u
#define IIC_ADDR_FRAM_DEV5		0x55u
#define IIC_ADDR_FRAM_DEV6		0x56u
#define IIC_ADDR_FRAM_DEV7		0x57u

#define IIC_ADDR_PULSE			0x57u

#define IIC_ADDR_RANGE			0x29u

#define IIC_ADDR_RD_MASK		0x01u
#define IIC_ADDR_SHIFT			1u

//t = 5us (meas fscl = 90 kHz)
#define IIC_RECOVERY_DELAY_SCL	75ul

#define IIC_ICR_VAL_400KHZ		0x11u //meas fscl = 400 kHz
#define IIC_ICR_VAL_100KHZ		0x1Fu //meas fscl = 97 kHz
#define IIC_ICR_VAL_10KHZ		0x3Du //meas fscl = 9.375 kHz

#define IIC_MAKE_ADDR_RD(addr)	(((addr) << IIC_ADDR_SHIFT) | IIC_ADDR_RD_MASK)
#define IIC_MAKE_ADDR_WR(addr)	((addr) << IIC_ADDR_SHIFT)

#endif /* _RES_ALLOC_IIC_H_ */
