/**
 * @Copyright 			(c) 2019,mculover666 All rights reserved	
 * @filename  			lcd_spi2_drv.c
 * @breif						Drive LCD based on spi2 commucation interface
 * @version
 *            			1.0    完成基本驱动程序，可以刷屏	mculover666    2019/7/10
 */

#ifndef _LCD_SPI2_DRV_H_
#define _LCD_SPI2_DRV_H_

#include "stm32l4xx_hal.h"

#define LCD_PWR_Pin 				GPIO_PIN_15
#define LCD_PWR_GPIO_Port 	GPIOB
#define LCD_WR_RS_Pin 			GPIO_PIN_6
#define LCD_WR_RS_GPIO_Port GPIOC
#define LCD_RST_Pin 				GPIO_PIN_7
#define LCD_RST_GPIO_Port 	GPIOC

#define	LCD_PWR(n)		(n?HAL_GPIO_WritePin(LCD_PWR_GPIO_Port,LCD_PWR_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(LCD_PWR_GPIO_Port,LCD_PWR_Pin,GPIO_PIN_RESET))
#define	LCD_WR_RS(n)	(n?HAL_GPIO_WritePin(LCD_WR_RS_GPIO_Port,LCD_WR_RS_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(LCD_WR_RS_GPIO_Port,LCD_WR_RS_Pin,GPIO_PIN_RESET))
#define	LCD_RST(n)		(n?HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_RESET))
#define LCD_Width			240
#define LCD_Height		240

//画笔颜色
#define WHITE         0xFFFF	//白色
#define YELLOW        0xFFE0	//黄色
#define BRRED 			  0XFC07  //棕红色
#define PINK          0XF81F	//粉色
#define RED           0xF800	//红色
#define BROWN 			  0XBC40  //棕色
#define GRAY  			  0X8430  //灰色
#define GBLUE			 		0X07FF	//兰色
#define GREEN         0x07E0	//绿色
#define BLUE         	0x001F  //蓝色
#define BLACK         0x0000	//黑色



void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void LCD_Clear(uint16_t color);
void LCD_Init(void);			//LCD初始化函数


#endif /* _LCD_SPI2_DRV_H_ */
