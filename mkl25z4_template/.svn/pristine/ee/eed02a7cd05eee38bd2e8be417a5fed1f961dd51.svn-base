/*
 * Name: acc.h
 * Author: Martin Stankus
 *
 */

#ifndef _ACC_H_
#define _ACC_H_

#include <stdint.h>

#define ACC_ADDR_SAMP_BLK			0x01u

#define ACC_ADDR_WHOAMI				0x0Du
#define ACC_ADDR_XYZDATCFG			0x0Eu
#define ACC_ADDR_CR1				0x2Au
#define ACC_ADDR_CR2				0x2Bu
#define ACC_ADDR_CR4				0x2Du
#define ACC_ADDR_CR5				0x2Eu

#define ACC_WHOAMI_MMA8451Q			0x1Au
#define ACC_WHOAMI_FXLS8471Q		0x6Au

#define ACC_XYZDATCFG_2G			0x00u
#define ACC_XYZDATCFG_4G			0x01u
#define ACC_XYZDATCFG_8G			0x02u

#define ACC_CR1_STDB				0x00u
#define ACC_CR1_001_56HZ			0x39u
#define ACC_CR1_006_25HZ			0x31u
#define ACC_CR1_012_50HZ			0x29u
#define ACC_CR1_050_00HZ			0x21u
#define ACC_CR1_100_00HZ			0x19u
#define ACC_CR1_200_00HZ			0x11u
#define ACC_CR1_400_00HZ			0x09u
#define ACC_CR1_800_00HZ			0x01u

#define ACC_CR2_RST					0x40u

#define ACC_CR4_INT_EN_DRDY			0x01u

#define ACC_CR5_INT_DRDY_INT1		0x01u
#define ACC_CR5_INT_DRDY_INT2		0x00u

#define ACC_SAMP_BLK_LEN			6u

#define ACC_SPI_CTRL0_WR_MASK		0x80u
#define ACC_SPI_CTRL0_ADDR_MASK		0x7Fu
#define ACC_SPI_CTRL1_ADDR_MASK		0x80u

#define ACC_SPI_MAKE_CTRL0_RD(addr) ((addr) & ACC_SPI_CTRL0_ADDR_MASK)
#define ACC_SPI_MAKE_CTRL0_WR(addr)	((addr) | ACC_SPI_CTRL0_WR_MASK)
#define ACC_SPI_MAKE_CTRL1(addr)	((addr) & ACC_SPI_CTRL1_ADDR_MASK)

typedef struct __attribute__ ((packed)) {
	uint8_t axis_x_hi;
	uint8_t axis_x_lo;
	uint8_t axis_y_hi;
	uint8_t axis_y_lo;
	uint8_t axis_z_hi;
	uint8_t axis_z_lo;
} ACC_SAMP_BLK;

typedef struct __attribute__ ((packed)) {
	uint16_t rsvd;
	ACC_SAMP_BLK samp_blk;
} ACC_SPI_BULK_SAMP_BLK_RX;

typedef struct __attribute__ ((packed)) {
	uint8_t ctrl0;
	uint8_t ctrl1;
	uint8_t rsvd[ACC_SAMP_BLK_LEN];
} ACC_SPI_BULK_SAMP_BLK_TX;

#endif /* _ACC_H_ */
