/**
 * @Copyright 			(c) 2019,mculover666 All rights reserved	
 * @filename  			led.c
 * @breif						independent LED driver file
 * @version
 *            			1.0    mculover666    2019/7/10
 */

#include "led_drv.h"

/**
  * @brief  LED GPIO Initialization Function
  * @param  None
  * @retval None
  */
void LED_GPIO_init(void)
{
	
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LED_CLK();

  /*Configure GPIO pin*/
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
	
	/*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}
