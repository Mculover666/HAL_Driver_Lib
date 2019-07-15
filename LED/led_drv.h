#ifndef	_LED_DRV_H_
#define _LED_DRV_H_
/**
 * @Copyright (c) 2019,mculover666 
              All rights reserved	
 * @filename  led.h
 * @breif     independent LED driver file
 * @version
 *            1.0    mculover666    2019/7/10
 */

#include "stm32l4xx_hal.h"

#define LED_CLK()   __HAL_RCC_GPIOC_CLK_ENABLE();
#define LED_PORT	GPIOC
#define	LED_PIN		GPIO_PIN_13

#define LED(n)	 (n?HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET):HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET));

void LED_GPIO_init(void);

#endif /* _LED_DRV_H_ */
