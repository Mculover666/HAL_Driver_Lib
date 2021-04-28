/**
 * @Copyright   (c) 2019,mculover666 All rights reserved	
 * @filename    at24C02_i2c_drv.c
 * @breif       Drive AT24C02(EEPROM) based on stm32 iic peripheral
 * @changelog   v1.0    mculover666     2019/7/15
 *              v1.1    mculover666     2021/4/15   (add WRITE_CYCLE_TIME_MS)
 *              v1.2    mculover666     2021/4/28   (add value operation)
 */

#ifndef	_AT24C02_I2C_DRV_H_
#define	_AT24C02_I2C_DRV_H_

#include "stm32l4xx_hal.h"

/* slave device address of eeprom */
#define	AT24C02_ADDR_WRITE	0xA0
#define	AT24C02_ADDR_READ	0xA1

/* delay time between write and write or between write and read, value is 5-10ms generally */
#define WRITE_CYCLE_TIME_MS 5


int AT24C02_Write_Byte(uint16_t addr, uint8_t* dat);
int AT24C02_Read_Byte(uint16_t addr, uint8_t* read_buf);
int AT24C02_Write_Amount_Byte(uint16_t addr, uint8_t* dat, uint16_t size);
int AT24C02_Read_Amount_Byte(uint16_t addr, uint8_t* recv_buf, uint16_t size);
int AT24C02_Write_Value(uint16_t addr, void *ptr, uint16_t size);
int AT24C02_Read_Value(uint16_t addr, void *ptr, uint16_t size);

#endif	/* _AT24C02_I2C_DRV_H_ */
