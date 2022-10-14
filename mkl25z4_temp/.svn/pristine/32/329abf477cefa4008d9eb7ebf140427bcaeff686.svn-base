/*
 * Name: iic_bm.c
 * Author: Martin Stankus
 *
 */

#include "MKL25Z4.h"

#include "soc_def.h"
#include "res_alloc.h"

#include "iic_bm.h"
#include "delay.h"

/*
#define OD_PIN_SET_HIGH(GPIO, IOIND)	BME_LAC1_W(&((GPIO)->PDDR), IOIND)
#define OD_PIN_SET_LOW(GPIO, IOIND)		BME_LAS1_W(&((GPIO)->PDDR), IOIND)

#define PIN_GET(GPIO, IOIND)			BME_UBFX_W(&((GPIO)->PDIR), IOIND, 1u)
*/

#define OD_PIN_SET_HIGH(GPIO, IOMASK)	((GPIO)->PDDR &= ~(IOMASK))
#define OD_PIN_SET_LOW(GPIO, IOMASK)	((GPIO)->PDDR |= (IOMASK))

#define PIN_GET(GPIO, IOMASK)			((GPIO)->PDIR & (IOMASK))

uint8_t iic_recovery(GPIO_Type *gpio_scl, GPIO_Type *gpio_sda, uint32_t iomask_scl, uint32_t iomask_sda)
{
	uint8_t rec_cnt = 0u;

	while (1) {
		rec_cnt++;

		OD_PIN_SET_HIGH(gpio_scl, iomask_scl);
		delay(IIC_RECOVERY_DELAY_SCL);

		if(!PIN_GET(gpio_scl, iomask_scl)) {
			rec_cnt = 0u;
			break;
		}

		if (PIN_GET(gpio_sda, iomask_sda)) {
			break;
		}

		if (rec_cnt == 9u) {
			rec_cnt = 0u;
			break;
		}

		OD_PIN_SET_LOW(gpio_scl, iomask_scl);
		delay(IIC_RECOVERY_DELAY_SCL);
	}

	return rec_cnt;
}

void iic_init(I2C_Type *iic, uint8_t icr_val)
{
	iic->F = I2C_F_ICR(icr_val);
}

void iic_bus_start(I2C_Type *iic)
{
	iic->C1 = I2C_C1_IICEN_MASK | I2C_C1_MST_MASK | I2C_C1_TX_MASK;
}

void iic_bus_repstart(I2C_Type *iic)
{
	iic->C1 = I2C_C1_IICEN_MASK | I2C_C1_MST_MASK | I2C_C1_TX_MASK | I2C_C1_RSTA_MASK;
}

void iic_bus_stop(I2C_Type *iic)
{
	iic->C1 = I2C_C1_IICEN_MASK;
	while (iic->S & I2C_S_BUSY_MASK);
}

uint8_t iic_bus_rx(I2C_Type *iic, uint8_t last)
{
	if (last) {
		iic->C1 = I2C_C1_IICEN_MASK | I2C_C1_MST_MASK | I2C_C1_TXAK_MASK;
	} else {
		iic->C1 = I2C_C1_IICEN_MASK | I2C_C1_MST_MASK;
	}

	iic->D;

	while (!(iic->S & I2C_S_IICIF_MASK));
	iic->S = I2C_S_IICIF_MASK;

	iic->C1 = I2C_C1_IICEN_MASK | I2C_C1_MST_MASK | I2C_C1_TX_MASK;

	return iic->D;
}

uint8_t iic_bus_tx(I2C_Type *iic, uint8_t payload)
{
	iic->D = payload;

	while (!(iic->S & I2C_S_IICIF_MASK));
	iic->S = I2C_S_IICIF_MASK;

	if (iic->S & I2C_S_RXAK_MASK) {
		return 1u;
	}

	return 0u;
}
