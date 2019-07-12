/**
 * @Copyright 			(c) 2019,mculover666 All rights reserved	
 * @filename  			lcd_spi2_drv.c
 * @breif				Drive LCD based on spi2 commucation interface
 * @version
 *            			v1.0    完成基本驱动程序，可以刷屏		        mculover666    2019/7/10
 *                      v1.1    添加打点、画线、画矩形、画圆算法实现     mculover666    2019/7/12
 *                      v1.2    添加显示英文ASCII字符和字符串           mculover666    2019/7/12
 *                      v1.3    添加绘制六芒星函数（基于画线函数）       mculover666    2019/7/12    
 * @note                移植说明（非常重要）：
 *                      1. LCD_SPI_Send是LCD的底层发送函数，如果是不同的芯片或者SPI接口，使用CubeMX生成初始化代码，
 *                         先修改此"lcd_spi2_drv.h"的LCD控制引脚宏定义，
 *                         然后修改LCD_SPI_Send中的内容即可；
 *                      2. 如果使用的是ST7789V2液晶控制器，但是不同分辨率的屏幕，修改"lcd_spi2_drv.h"中的LCD_Width和LCD_Height宏定义即可；
 *                      3. LCD_Buf_Size请勿轻易修改，会影响几乎所有的函数，除非你明确的了解后果；
 *                      4. 此驱动程序需要spi.h和spi.c的支持；
 *                      4. 其余情况不适配此驱动代码。
 */

#ifndef _LCD_SPI2_DRV_H_
#define _LCD_SPI2_DRV_H_

#include "stm32l4xx_hal.h"

#define LCD_PWR_Pin 		GPIO_PIN_15
#define LCD_PWR_GPIO_Port 	GPIOB
#define LCD_WR_RS_Pin 		GPIO_PIN_6
#define LCD_WR_RS_GPIO_Port GPIOC
#define LCD_RST_Pin 		GPIO_PIN_7
#define LCD_RST_GPIO_Port 	GPIOC

#define	LCD_PWR(n)		(n?HAL_GPIO_WritePin(LCD_PWR_GPIO_Port,LCD_PWR_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(LCD_PWR_GPIO_Port,LCD_PWR_Pin,GPIO_PIN_RESET))
#define	LCD_WR_RS(n)	(n?HAL_GPIO_WritePin(LCD_WR_RS_GPIO_Port,LCD_WR_RS_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(LCD_WR_RS_GPIO_Port,LCD_WR_RS_Pin,GPIO_PIN_RESET))
#define	LCD_RST(n)		(n?HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_RESET))
#define LCD_Width		240
#define LCD_Height		240

//画笔颜色
#define WHITE         0xFFFF	//白色
#define YELLOW        0xFFE0	//黄色
#define BRRED 		  0XFC07    //棕红色
#define PINK          0XF81F	//粉色
#define RED           0xF800	//红色
#define BROWN 		  0XBC40    //棕色
#define GRAY  		  0X8430    //灰色
#define GBLUE		  0X07FF	//兰色
#define GREEN         0x07E0	//绿色
#define BLUE          0x001F    //蓝色
#define BLACK         0x0000	//黑色



void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void LCD_Clear(uint16_t color);
void LCD_Init(void);			//LCD初始化函数
void LCD_Draw_ColorPoint(uint16_t x, uint16_t y,uint16_t color);
void LCD_Draw_ColorLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_Draw_ColorRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_Draw_ColorCircle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);
void LCD_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_ShowChar(uint16_t x, uint16_t y, char ch, uint16_t back_color, uint16_t font_color, uint8_t font_size);
void LCD_ShowCharStr(uint16_t x, uint16_t y, uint8_t max_width, char* str, uint16_t back_color, uint16_t font_color, uint8_t font_size);
void LCD_Draw_ColorSixPointStar(uint16_t x, uint16_t y, uint8_t r, uint16_t color);

#endif /* _LCD_SPI2_DRV_H_ */
