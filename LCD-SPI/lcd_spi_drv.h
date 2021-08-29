/**
 * @Copyright 			(c) 2019,mculover666 All rights reserved	
 * @filename  			lcd_spi_drv.h
 * @breif				Drive ST7789 LCD based on spi interface
 * @changelog
 *            			v1.0    finish basic function               mculover666    2019/7/10
 *                      v2.0    add macro define to control build   mculover666    2019/7/13
 *                      v2.1    add support for scroll function     mculover666    2021/5/18
 *                      v2.2    optimizing code style               mculover666    2021/5/19
 *                      v2.3    optimizing speed                    mculover666    2021/8/29
 */

#ifndef _LCD_SPI_DRV_H_
#define _LCD_SPI_DRV_H_

#include "stm32l4xx_hal.h"
#include "spi.h"

#define	USE_ASCII_FONT_LIB			1   // Whether to enable character display
#define USE_VERTICAL_SCROLL         0   // Whether to enable vertical scroll

#define LCD_WIDTH		240
#define LCD_HEIGHT		240

#define CHINESE_FONT_BUF_MAX_SIZE_ONE_CHR    128

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

typedef struct lcd_color_params_st {
    uint16_t    background_color;
    uint16_t    foreground_color;
} lcd_color_params_t;

/**
 * @brief	LCD Init.
 * @param   none
 * @return  none
 */
void lcd_init(void);

/**
 * @breif	turn on LCD power.
 * @param   none
 * @return  none
 */
void lcd_display_on(void);

/**
 * @brief	turn off LCD power.
 * @param   none
 * @return  none
 */
void lcd_display_off(void);

/**
 * @brief   Convert the RGB 565 color to hex.
 * @param   r   red value of color(0 - 63)
 * @param   g   green value of color(0 - 127)
 * @param   b   GREEN value of color(0 - 63)
 * @return  hex value of color
 * @note    color format: rgb 565
 * 
*/
uint16_t rgb2hex_565(uint16_t r, uint16_t g, uint16_t b);

/**
 * @brief	Set the background color and foreground color.
 * @param   back_color  rgb565
 * @param   fore_color  rgb565
 * @return  none
 */
void lcd_color_set(uint16_t back_color, uint16_t fore_color);

/**
 * @brief	clear LCD.
 * @param   none
 * @return  none
 */
void lcd_clear(void);

/**
 * @brief   draw a point with color.
 * @param   x,y     point address
 * @param   color   point color
 * @return  none
 */
void lcd_draw_point(uint16_t x, uint16_t y,uint16_t color);

/**
 * @brief   draw a line with color.
 * @param   x1,y1   line start address
 * @param   x2,y2   line end address
 * @param   color   line color
 * @return  none
 */
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/**
 * @brief   draw a rect with color.
 * @param   x1,y1   rect start address
 * @param   x2,y2   rect end address
 * @param   color   rect color
 * @return  none
 */
void lcd_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/**
 * @brief   fill a rect with color.
 * @param   x1,y1   rect start address
 * @param   x2,y2   rect end address
 * @param   color   rect color
 * @return  none
 */
void lcd_fill_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/**
 * @brief   clear a rect.
 * @param   x1,y1   rect start address
 * @param   x2,y2   rect end address
 * @return  none
 */
void lcd_clear_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

/**
 * @brief   draw a circle with color.
 * @param   x,y     center of circle address
 * @param   r       radius of cirlce
 * @param   color   circle color
 * @return  none
 */
void lcd_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);

/**
 * @brief   display a character with color.
 * @param   x1,y1       character start address
 * @param   ch          character
 * @param   font_size   font size of character
 * @return  none
 */
void lcd_draw_char(uint16_t x, uint16_t y, char ch, uint8_t font_size);

/**
 * @brief       Show a chinese char.
 * @param[in]   x1  horizontal start position.
 * @param[in]   y1  vertical start position.
 * @param[in]   x2  horizontal end position.
 * @param[in]   y2  vertical end position.
 * @param[in]   offset      offset in hz library.
 * @param[in]   font_size   support 16.
 * @return      None
 * @note        This function need hz library.
*/
void lcd_draw_chinese(uint16_t x, uint16_t y, uint32_t offset, uint8_t font_size);

/**
 * @brief   display a string with color.
 * @param   x,y         string start address
 * @param   str         string
 * @param   font_size   font size of character
 * @return  none
 */
void lcd_draw_text(uint16_t x, uint16_t y, char* str, uint8_t font_size);

/**
 * @brief   display a picture.
 * @param   x,y         picture start address
 * @param   width       width of picture
 * @param   height      height of pictue
 * @param   p           pointer to picture buffer
 * @return  none
 */
void lcd_show_image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *p);

#if USE_VERTICAL_SCROLL
int lcd_set_scroll_area(uint16_t tfa, uint16_t vsa, uint16_t bta);
void lcd_set_scroll_start_address(uint16_t vsp);
#endif /* USE_VERTICAL_SCROLL */

#endif /* _LCD_SPI_DRV_H_ */
