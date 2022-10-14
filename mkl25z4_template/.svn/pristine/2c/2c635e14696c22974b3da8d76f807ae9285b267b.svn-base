/*
 * Name: iic_bm.c
 * Author: Martin Stankus
 *
 */

#ifndef _IIC_BM_H_
#define _IIC_BM_H_

#include "MKL25Z4.h"

uint8_t iic_recovery(GPIO_Type *gpio_scl, GPIO_Type *gpio_sda, uint32_t iomask_scl, uint32_t iomask_sda);
void iic_init(I2C_Type *iic, uint8_t icr_val);

void iic_bus_start(I2C_Type *iic);
void iic_bus_repstart(I2C_Type *iic);
void iic_bus_stop(I2C_Type *iic);
uint8_t iic_bus_rx(I2C_Type *iic, uint8_t last);
uint8_t iic_bus_tx(I2C_Type *iic, uint8_t payload);

#endif /* _IIC_BM_H_ */
