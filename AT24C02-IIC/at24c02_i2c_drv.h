/**
 * @Copyright 			(c) 2019,mculover666 All rights reserved	
 * @filename  			at24c02_i2c_drv.h
 * @breif				Drive AT24C02(EEPROM) based on iic1 commucation interface
 * @version
 *            			v1.0    完成基本驱动程序              mculover666    2019/7/15
 * @note                移植说明（非常重要）：	
 *						1. 修改at24c02_i2c_drv.h文件中的AT24C02读写地址即可；
 * 					    2. 此驱动程序需要STM32CubeMX生成的I2C初始化文件i2c.h和i2c.c支持。
 */
#ifndef	_AT24C02_I2C_DRV_H_
#define	_AT24C02_I2C_DRV_H_

#include "stm32l4xx_hal.h"

#define	AT24C02_ADDR_WRITE	0xA0
#define	AT24C02_ADDR_READ	0xA1

uint8_t At24c02_Write_Byte(uint16_t addr, uint8_t* dat);
uint8_t At24c02_Read_Byte(uint16_t addr, uint8_t* read_buf);
uint8_t At24c02_Write_Amount_Byte(uint16_t addr, uint8_t* dat, uint16_t size);
uint8_t At24c02_Read_Amount_Byte(uint16_t addr, uint8_t* recv_buf, uint16_t size);

#endif	/* _AT24C02_I2C_DRV_H_ */
