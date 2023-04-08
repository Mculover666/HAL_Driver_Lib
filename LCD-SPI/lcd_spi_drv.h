/**
 * @Copyright   (c) 2019,mculover666 All rights reserved
 * @filename    lcd_spi_drv.c
 * @breif       Drive ST7789 LCD based on spi interface
 * @changelog
 *              v1.0    finish basic function                   mculover666     2019/7/10
 *              v2.0    add macro define to control build       mculover666     2019/7/13
 *              v2.1    add support for scroll function         mculover666     2021/5/18
 *              v2.2    optimize code style                     mculover666     2021/5/19
 *              v2.3    optimize speed(remove buffer)           mculover666     2021/8/29
 *              V2.4    optimize lcd_draw_chinese_char          mculover666     2022/3/11
 *              v2.5    optimize speed(register send)           Yangyuanxin     2022/6/26
 *              v2.6    optimize port interface                 mculover666     2022/7/17
 *              v2.7    add spi send with buffer(very useful)   mculover666     2022/7/18
 *              v2.8    port on esp-idf platform            	mculover666    	2023/4/8
 */


#ifndef _LCD_SPI_DRV_H_
#define _LCD_SPI_DRV_H_

#include <stdint.h>

/* platform config */
#define STM32_PLATFORM
//#define ESP32_PLATFORM

/* screen config */
#define LCD_WIDTH		240
#define LCD_HEIGHT		240
#define LCD_DIRECTION   0               // 0: normal, 1: left 90, 2: 180, 3: right 90.

/* feature config */
#define	USE_ASCII_FONT_LIB          1   // Whether to enable character display
#define USE_VERTICAL_SCROLL         0   // Whether to enable vertical scroll
#define USE_DISPLAY_INVERSION       1   // Whether to enable display inversion
#define USE_SPI_WRITE_BUF           1   // Whether to enable spi write through buffer

/* chinese font interior config */
#define CHINESE_FONT_BUF_MAX_SIZE_ONE_CHR    128

#define USE_REGISTER_METHOD         0

#if USE_SPI_WRITE_BUF
#define SPI_WRITE_BUFFER_SIZE 128
#endif

/*
    some useful color, rgb565 format.
    of courcse, you can use rgb2hex_565 API.
*/
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

typedef struct lcd_spi_drv_st {
    //the function to operate gpio.
    int (*cs)(int status);
    int (*blk)(int status);
    int (*rst)(int status);
    int (*dc)(int status);

    //the function to operate spi.
    int (*init)(void);
    int (*write_byte)(uint8_t data);
    int (*write_multi_bytes)(uint8_t *data, uint16_t size);

    ///the function to delay ms.
    void (*delay)(uint32_t ms);
} lcd_spi_drv_t;

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
 * @brief   fill a rect with color buffer.
 * @param   x1,y1   start address
 * @param   x2,y2   end address
 * @param   color   the pointer of color buffer
 * @return  none
 */
void lcd_fill_with_buffer(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color);

/**
 * @brief	clear LCD.
 * @param   none
 * @return  none
 */
void lcd_clear(void);

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
 * @param   x,y         character start address
 * @param   ch          character
 * @param   font_size   font size of character
 * @return  none
 */
void lcd_draw_char(uint16_t x, uint16_t y, char ch, uint8_t font_size);

/**
 * @brief   show a chinese char.
 * @param   x,y         character start address
 * @param   font_width  the width of chinese char.
 * @param   font_height the height of chinese char.
 * @param   font_data   the data of chinese char form font lib.
 * @return  None
*/
void lcd_draw_chinese_char(uint16_t x, uint16_t y, uint8_t font_width, uint8_t font_height, uint8_t *font_data);

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
