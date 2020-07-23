/**********************************************************
 *@file    sgp30.h
 *@brief
 *         基于STM32 HAL 库的SGP30 甲醛传感器驱动
 *
 *@author  mculover666<2412828003@qq.com>
 *@date    2020/07/23
 *@note
 *         1. 驱动默认使用硬件i2c 1
 *         2. 如果不需要打印报错信息，可以去掉printf相关
**********************************************************/

#ifndef _SGP30_H_
#define _SGP30_H_

#include "stm32l4xx.h"
#include <stdio.h>

#define SGP30_ADDR          0x58
#define	SGP30_ADDR_WRITE	SGP30_ADDR<<1       //0xb0
#define	SGP30_ADDR_READ		(SGP30_ADDR<<1)+1   //0xb1

/* SGP30传感器接的硬件I2C接口 */
#define SGP30_I2C_Handle_Name   hi2c1

typedef struct sgp30_data_st {
    uint16_t co2;
    uint16_t tvoc;
}sgp30_data_t;

typedef enum sgp30_cmd_en {
    /* 初始化空气质量测量 */
    INIT_AIR_QUALITY = 0x2003,
    
    /* 开始空气质量测量 */
    MEASURE_AIR_QUALITY = 0x2008
    
} sgp30_cmd_t;

extern I2C_HandleTypeDef SGP30_I2C_Handle_Name;
extern sgp30_data_t      sgp30_data;

int sgp30_init(void);
int sgp30_read(void);


#endif /* _SGP30_H_ */
