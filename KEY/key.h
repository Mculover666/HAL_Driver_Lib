/**
 * @Copyright 			(c) 2019,mculover666 All rights reserved	
 * @filename  			key.h
 * @breif				independent KEY driver file
 * @version
 *            			v1.0    mculover666    2019/7/15
 */
#ifndef _KEY_H_
#define _KEY_H_

#include "stm32l4xx_hal.h"

#define	KEY1_CLK()      __HAL_RCC_GPIOB_CLK_ENABLE();
#define KEY1_Pin        GPIO_PIN_2
#define KEY1_GPIO_Port  GPIOB
#define	KEY2_CLK()      __HAL_RCC_GPIOB_CLK_ENABLE();
#define KEY2_Pin        GPIO_PIN_3
#define KEY2_GPIO_Port  GPIOB


void KEY_GPIO_Init(void);

#endif /* _KEY_H_ */
