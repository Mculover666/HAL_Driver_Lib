/**
 * @Copyright 			(c) 2019,mculover666 All rights reserved	
 * @filename  			lcd_spi_drv.h
 * @breif				Drive ST7789 LCD based on spi interface
 * @changelog
 *            			v1.0    finish baic function                mculover666    2019/7/10
 *                      v2.0    add macro define to control build   mculover666    2019/7/13
 *                      v2.1    add support for scroll function     mculover666    2021/5/18
 *                      v2.2    optimizing code style               mculover666    2021/5/19
 */

#ifndef _LCD_SPI_DRV_H_
#define _LCD_SPI_DRV_H_

#include "stm32l4xx_hal.h"
#include "spi.h"

#define	USE_ASCII_FONT_LIB			1   // Whether to enable character display
#define USE_PICTURE_DISPLAY			0   // Whether to enable picture display
#define USE_VERTICAL_SCROLL         0   // Whether to enable vertical scrolling

#define LCD_Width		240
#define LCD_Height		240

#define LCD_PWR_Pin 		GPIO_PIN_15
#define LCD_PWR_GPIO_Port 	GPIOB
#define LCD_WR_RS_Pin 		GPIO_PIN_6
#define LCD_WR_RS_GPIO_Port GPIOC
#define LCD_RST_Pin 		GPIO_PIN_7
#define LCD_RST_GPIO_Port 	GPIOC
#define LCD_SPI_HANDLER     hspi2

#define	LCD_PWR(n)		(n?HAL_GPIO_WritePin(LCD_PWR_GPIO_Port,LCD_PWR_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(LCD_PWR_GPIO_Port,LCD_PWR_Pin,GPIO_PIN_RESET))
#define	LCD_WR_RS(n)	(n?HAL_GPIO_WritePin(LCD_WR_RS_GPIO_Port,LCD_WR_RS_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(LCD_WR_RS_GPIO_Port,LCD_WR_RS_Pin,GPIO_PIN_RESET))
#define	LCD_RST(n)		(n?HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(LCD_RST_GPIO_Port,LCD_RST_Pin,GPIO_PIN_RESET))

#define WHITE         0xFFFF	
#define YELLOW        0xFFE0	
#define BRRED 		  0XFC07
#define PINK          0XF81F	
#define RED           0xF800	
#define BROWN 		  0XBC40    
#define GRAY  		  0X8430    
#define GBLUE		  0X07FF	
#define GREEN         0x07E0	
#define BLUE          0x001F   
#define BLACK         0x0000

#define DEFAULT_COLOR_AFTER_INIT   WHITE 

void lcd_display_on(void);
void lcd_display_off(void);
void lcd_clear(uint16_t color);
void lcd_init(void);
void lcd_draw_color_point(uint16_t x, uint16_t y,uint16_t color);
void lcd_draw_color_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcd_draw_color_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcd_draw_color_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);
void lcd_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
uint16_t rgb2hex_565(uint16_t r, uint16_t g, uint16_t b);

#if USE_ASCII_FONT_LIB
void lcd_show_char(uint16_t x, uint16_t y, char ch, uint16_t back_color, uint16_t font_color, uint8_t font_size);
void lcd_show_str(uint16_t x, uint16_t y, char* str, uint16_t back_color, uint16_t font_color, uint8_t font_size);
#endif /* USE_ASCII_FONT_LIB */

void lcd_draw_color_sixpointstar(uint16_t x, uint16_t y, uint8_t r, uint16_t color);

#if USE_PICTURE_DISPLAY
void lcd_show_image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *p);
#endif /*USE_PICTURE_DISPLAY */

#if USE_VERTICAL_SCROLL
int lcd_set_scroll_area(uint16_t tfa, uint16_t vsa, uint16_t bta);
void lcd_set_scroll_start_address(uint16_t vsp);
#endif /* USE_VERTICAL_SCROLL */

#endif /* _LCD_SPI_DRV_H_ */
