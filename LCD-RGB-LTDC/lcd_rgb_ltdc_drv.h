#ifndef _LCD_RGB_LTDC_DRV_H_
#define _LCD_RGB_LTDC_DRV_H_

#include "ltdc.h"

/**
 * @brief   Windows size on lcd.
*/
#define LCD_WIDTH       1024
#define LCD_HEIGHT      600

/**
 * @brief   Backlight control pin of lcd.
*/
#define LCD_BL_GPIO_PORT    GPIOB
#define LCD_BL_GPIO_PIN     GPIO_PIN_5

/**
 * @brief   start address of lcd framebuffer.
*/
#define LCD_FRAME_BUFFER    0xc0000000

/**
 * @brief   whether use dma2d to transfer data to lcd framebuffer.
*/
#define USE_DMA2D_EN        1

/**
 * @brief   color
 * @note    rgb565   
*/
#define BLACK   0x0000
#define BLUE    0x001F
#define GREEN   0x07E0
#define GBLUE   0X07FF
#define GRAY    0X8430
#define BROWN   0XBC40
#define RED     0xF800
#define PINK    0XF81F
#define BRRED   0XFC07
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

/**
 * @brief       Control the lcd backlight.
 * @param[in]   brightness  the value of lcd backlight.
 * @return      None
*/
void lcd_backlight_control(uint8_t bightness);

/**
 * @brief       LCD initialization.
 * @param       None
 * @return      None
*/
void lcd_init(void);

/**
 * @brief       Clear lcd.
 * @param[in]   color   rgb565
 * @return      None
*/
void lcd_clear(uint16_t color);

/**
 * @brief       Draw point.
 * @param[in]   x       horizontal position.
 * @param[in]   y       vertical position.
 * @param[in]   color   rgb565
 * @return      None
*/
void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief       Read point.
 * @param[in]   x       horizontal position.
 * @param[in]   y       vertical position.
 * @return      color   rgb565
*/
uint16_t lcd_read_point(uint16_t x, uint16_t y);

/**
 * @brief       Draw line.
 * @param[in]   x1      horizontal start position.
 * @param[in]   y1      vertical start position.
 * @param[in]   x2      horizontal end position.
 * @param[in]   y2      vertical end position.
 * @param[in]   color   rgb565
 * @return      None
 * @note        Bresenham algorithm.
*/
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/**
 * @brief       Draw rect.
 * @param[in]   x1      horizontal start position.
 * @param[in]   y1      vertical start position.
 * @param[in]   x2      horizontal end position.
 * @param[in]   y2      vertical end position.
 * @param[in]   color   rgb565
 * @return      None
*/
void lcd_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/**
 * @brief       Fill rect.
 * @param[in]   x1      horizontal start position.
 * @param[in]   y1      vertical start position.
 * @param[in]   x2      horizontal end position.
 * @param[in]   y2      vertical end position.
 * @param[in]   color   rgb565
 * @return      None
*/
void lcd_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

#endif /* _LCD_RGB_LTDC_DRV_H_ */