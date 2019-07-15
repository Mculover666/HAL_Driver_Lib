/**
 * @Copyright 			(c) 2019,mculover666 All rights reserved	
 * @filename  			key.c
 * @breif						independent KEY driver file
 * @version
 *            			v1.0    mculover666    2019/7/15
 */

#include "key.h"

/**
 * @brief		初始化两个按键的GPIO为输入
 * @param		none
 * @retval	none
*/

void KEY_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  KEY1_CLK();
	KEY2_CLK();

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

}
