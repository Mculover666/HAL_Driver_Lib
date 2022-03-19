#include "lcd_rgb_ltdc_drv.h"

#if USE_DMA2D_EN
#include "dma2d.h"
#endif /* USE_DMA2D_EN */

#if USE_ASCII_EN
#include "font/font_ascii.h"
#endif /* USE_ASCII_EN */

#if USE_CHINESE_EN
#if USE_CHINESE_FULL_LIB

#if CHINESE_FULL_LIB_16_EN
#include "font/hz16.h"
#endif
#if CHINESE_FULL_LIB_24_EN
#include "font/hz24.h"
#endif
#if  CHINESE_FULL_LIB_32_EN
#include "font/hz32.h"
#endif

#else
#include "font/font_hz.h"
#endif /* USE_CHINESE_FULL_LIB */
#endif /* USE_CHINESE_EN */

void lcd_backlight_control(uint8_t bightness)
{
    // todo: use pwm to control backlight

    if (bightness) {    
        // turn on the backlight
        HAL_GPIO_WritePin(LCD_BL_GPIO_PORT, LCD_BL_GPIO_PIN, GPIO_PIN_SET);
    } else {            
        // turn off the backlight
        HAL_GPIO_WritePin(LCD_BL_GPIO_PORT, LCD_BL_GPIO_PIN, GPIO_PIN_RESET);
    }
}

#if USE_DMA2D_EN
static void dma2d_transfer_data_r2m(uint32_t *addr, uint32_t xSize, uint32_t ySize, uint32_t offsetLine, uint16_t color)
{
    DMA2D->CR = DMA2D_R2M;   // dma2d mode: register to memory.
    DMA2D->OPFCCR = DMA2D_OUTPUT_RGB565;

    DMA2D->OCOLR = color;
    DMA2D->OMAR = (uint32_t)addr;
    DMA2D->OOR = offsetLine;
    DMA2D->NLR = (uint32_t)(xSize << 16) | (uint16_t)ySize;
    
    DMA2D->CR |= DMA2D_CR_START;
    while (DMA2D->CR & DMA2D_CR_START);
}

static void dma2d_transfer_data_m2m(uint32_t *addrDst, uint32_t xSize, uint32_t ySize, uint32_t offsetLineDst, uint32_t *addrSrc, uint32_t offsetLineSrc)
{
    DMA2D->CR = DMA2D_M2M;   // dma2d memory: memory to memory.
    DMA2D->OPFCCR = DMA2D_OUTPUT_RGB565;

    DMA2D->OMAR = (uint32_t)addrDst;
    DMA2D->FGMAR = (uint32_t)addrSrc;
    DMA2D->OOR = offsetLineDst;
    DMA2D->FGOR = offsetLineSrc;
    DMA2D->NLR = (uint32_t)(xSize << 16) | (uint16_t)ySize;
    
    DMA2D->CR |= DMA2D_CR_START;
    while (DMA2D->CR & DMA2D_CR_START);
}
#endif

void lcd_clear(uint16_t color)
{
#if USE_DMA2D_EN
    dma2d_transfer_data_r2m((uint32_t *)LCD_FRAME_BUFFER, LCD_WIDTH, LCD_HEIGHT, 0, color);
#else
    uint16_t *ptr = (uint16_t*)LCD_FRAME_BUFFER;
    uint32_t i = 0;

    while (i++ < LCD_WIDTH*LCD_HEIGHT) {
        *(ptr+i) = color;
    }
#endif /* USE_DMA2D_EN */
}

void lcd_init()
{
    lcd_clear(BLACK);
    lcd_backlight_control(255);
}

void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    uint32_t pos;
    uint16_t *ptr;

    // check position.
    if (x > LCD_WIDTH || y > LCD_HEIGHT) {
        return;
    }

    // calculate the position offset in framebuffer.
    pos = x + y*LCD_WIDTH;
    ptr = (uint16_t*)LCD_FRAME_BUFFER;

    // modify the framebuffer.
#if USE_DMA2D_EN
    dma2d_transfer_data_r2m((uint32_t *)(ptr+pos), 1, 1, 0, color);
#else
    *(ptr+pos) = color;
#endif /* USE_DMA2D_EN */
}

uint16_t lcd_read_point(uint16_t x, uint16_t y)
{
    uint32_t pos;
    uint16_t *ptr, data;

    // check position.
    if (x > LCD_WIDTH || y > LCD_HEIGHT) {
        return 0;
    }

    // calculate the position offset in framebuffer.
    pos = x + y*LCD_WIDTH;
    ptr = (uint16_t*)LCD_FRAME_BUFFER;

    // read the framebuffer.
    data = *(ptr+pos);

    return data;
}

void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t	t, x, y, x_temp, y_temp, distance;
	int16_t		delta_x = 0, delta_y = 0;
	int8_t		incx = 0, incy = 0;
	
    delta_x = x2 - x1;
    delta_y = y2 - y1;
    if (delta_x > 0) {          // slash, left to right.
        incx = 1;
    } else if (delta_x == 0) {  // vertical line.
        incx = 0;
    } else {                    //slash, right to left.
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0) {          // slash, left to right.
        incy = 1;
    } else if (delta_y == 0) {  // horizontal line.
        incy = 0;
    } else {                    //slash, right to left.
        incy = -1;
        delta_y = -delta_y;
    }			

    if (delta_x > delta_y) {
        distance = delta_x;
    } else {
        distance = delta_y;
    }
    
    x = x1;
    y = y1;
    for (t = 0; t <= distance + 1; t++) {
        lcd_draw_point(x, y, color);
        x_temp += delta_x;	
        if (x_temp > distance) {
            x_temp -= distance;		
            x += incx;
        }
        y_temp += delta_y;
        if(y_temp > distance)
        {
            y_temp -= distance;
            y += incy;
        }
    }
}

void lcd_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    lcd_draw_line(x1, y1, x2, y1, color);
	lcd_draw_line(x1, y1, x1, y2, color);
	lcd_draw_line(x1, y2, x2, y2, color);
	lcd_draw_line(x2, y1, x2, y2, color);
}

void lcd_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint32_t pos;
    uint16_t *ptr;
    uint16_t offsetLine, rect_width, rect_height;

    // check position.
    if (x1 > LCD_WIDTH || y1 > LCD_HEIGHT || x2 > LCD_WIDTH || y1 > LCD_HEIGHT || x1 > x2 || y1 > y2) {
        return;
    }

    rect_width = x2 - x1;
    rect_height = y2 - y1;

    // calculate the position offset in framebuffer.
    pos = x1 + y1*LCD_WIDTH;
    ptr = (uint16_t*)LCD_FRAME_BUFFER;
    offsetLine = LCD_WIDTH - rect_width;

#if USE_DMA2D_EN
    dma2d_transfer_data_r2m((uint32_t *)(ptr+pos), rect_width, rect_height, offsetLine, color);
#else
    uint16_t i,j;
    uint32_t offset;

    offset = 0;
    for (i = 0; i < rect_height; i++) {
        for (j = 0; j < rect_width; j++) {
            *(ptr+pos+offset) = color;
            offset++;
        }
        offset += offsetLine;
    }
#endif /* USE_DMA2D_EN */
}

void lcd_fill_with_buffer(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color)
{
    uint32_t pos;
    uint16_t *ptr;
    uint16_t offsetLine, rect_width, rect_height;

    // check position.
    if (x1 > LCD_WIDTH || y1 > LCD_HEIGHT || x2 > LCD_WIDTH || y1 > LCD_HEIGHT || x1 > x2 || y1 > y2) {
        return;
    }

    rect_width = x2 - x1 + 1;
    rect_height = y2 - y1 + 1;

    // calculate the position offset in framebuffer.
    pos = x1 + y1*LCD_WIDTH;
    ptr = (uint16_t*)LCD_FRAME_BUFFER;
    offsetLine = LCD_WIDTH - rect_width;

#if USE_DMA2D_EN
    dma2d_transfer_data_m2m((uint32_t *)(ptr+pos), rect_width, rect_height, offsetLine, (uint32_t *)color, 0);
#else
    uint16_t i,j;
    uint32_t offset;

    offset = 0;
    for (i = 0; i < rect_height; i++) {
        for (j = 0; j < rect_width; j++) {
            *(ptr+pos+offset) = *color++;
            offset++;
        }
        offset += offsetLine;
    }
#endif /* USE_DMA2D_EN */
}

#if USE_ASCII_EN
void lcd_show_char(uint16_t x, uint16_t y, char ch, uint16_t back_color, uint16_t font_color, uint8_t font_size)
{
    uint16_t i, j;
    uint16_t x_pos, y_pos, size, font_width, font_height;
    uint8_t *font_ptr;
    uint8_t bit_width, temp;
	
    if((x > (LCD_WIDTH - font_size / 2)) || (y > (LCD_HEIGHT - font_size)))	{
        return;
    }
	
    font_width = font_size / 2;
    font_height = font_size;
    size = (font_width / 8 + ((font_width % 8) ? 1 : 0)) * font_height;
    x_pos = x;
    y_pos = y;
    ch = ch - ' ';

    switch (font_size) {
        case 12:
            bit_width = 6;
            font_ptr = (uint8_t*)&asc2_1206[ch];
        case 16:
            bit_width = 8;
            font_ptr = (uint8_t*)&asc2_1608[ch];
            break;
        case 24:
            font_ptr = (uint8_t*)&asc2_2412[ch];
            break;
        case 32:
            bit_width = 8;
            font_ptr = (uint8_t*)&asc2_3216[ch];
            break;
        default:
            return;
    }
	 
    for (i = 0; i < size; i++) {
        temp = *(font_ptr + i);

        if (font_size == 24) {
            bit_width = (i % 2 == 0) ? 8 : 4;
        }

        for (j = 0; j < bit_width; j++) {
            if(temp & 0x80){
                lcd_draw_point(x_pos, y_pos, font_color);
            } else {
                lcd_draw_point(x_pos, y_pos, back_color);
            }
            temp <<= 1;
            x_pos++;
        }
        if (x_pos >= (x + font_width)) {
            y_pos++;
            x_pos = x;
        }
    }
}

#endif /* USE_ASCII_EN */

#if USE_CHINESE_EN
void lcd_show_chinese(uint16_t x, uint16_t y, uint32_t offset, uint16_t back_color, uint16_t font_color, uint8_t font_size)
{
    uint16_t i, j;
    uint16_t x_pos, y_pos, size, font_width, font_height;
    uint8_t *font_ptr;
    uint8_t bit_width, temp;
	
    if((x > (LCD_WIDTH - font_size)) || (y > (LCD_HEIGHT - font_size)))	{
        return;
    }
	
    x_pos = x;
    y_pos = y;
    font_height = font_size;
    font_width = font_size;
    bit_width = 8;
    size = (font_width / 8 + ((font_width % 8) ? 1 : 0)) * font_height;

    switch (font_size) {
        case 16:
#if USE_CHINESE_FULL_LIB
#if CHINESE_FULL_LIB_16_EN
            font_ptr = (uint8_t*)(hz16) + offset;
#else
            font_ptr = NULL;
#endif
#else
            font_ptr = (uint8_t*)&hz_16x16[offset];
#endif
            break;
        case 24:
#if USE_CHINESE_FULL_LIB
#if CHINESE_FULL_LIB_24_EN
            font_ptr = (uint8_t*)(hz24) + offset;
#else
            font_ptr = NULL;
#endif 
#else
            font_ptr = (uint8_t*)&hz_24x24[offset];
#endif
            break;
        case 32: 
#if USE_CHINESE_FULL_LIB
#if CHINESE_FULL_LIB_32_EN
            font_ptr = (uint8_t*)(hz32) + offset;
#else
            font_ptr = NULL;
#endif
#else
            font_ptr = (uint8_t*)&hz_32x32[offset];
#endif
            break;
        default:
            return;
    }
    
    if (!font_ptr) {
        return;
    }
    
    for (i = 0; i < size; i++) {
        temp = *(font_ptr + i);
        for (j = 0; j < bit_width; j++) {
            if(temp & 0x80){
                lcd_draw_point(x_pos, y_pos, font_color);
            } else {
                lcd_draw_point(x_pos, y_pos, back_color);
            }
            temp <<= 1;
            x_pos++;
        }
        if (x_pos >= (x + font_width)) {
            y_pos++;
            x_pos = x;
        }
    }
}
#endif /* USE_CHINESE_EN */

#if USE_ASCII_EN || USE_CHINESE_EN

#include <stdio.h>

static uint32_t gb2312_calc_offset(char *code, uint8_t font_size)
{
    uint8_t qh, ql;
    uint32_t offset;
    uint8_t font_bytes;
    
    font_bytes = (font_size / 8 + ((font_size % 8) ? 1 : 0)) * font_size;
    
    qh = *code;
    ql = *(code + 1);
    
    if (qh >= 0xA1 && qh <= 0xA9 && ql >=0xA1) {
        offset = (94*(qh - 0xA1) + (ql - 0xA1)) * font_bytes;
    } else if (qh >=0xB0 && qh <= 0xF7 && ql >=0xA1) {
        offset = (94*(qh - 0xB0) + (ql - 0xA1) + 846) * font_bytes;
    }
    
    return offset;
}
    
void lcd_show_str(uint16_t x, uint16_t y, char *str, uint16_t back_color, uint16_t font_color, uint8_t font_size)
{
    uint16_t font_width;
    uint16_t x_pos = x;
    uint32_t offset;

    while (*str) {
        if (*str < 0x80) {
            lcd_show_char(x_pos, y, *str, back_color, font_color, font_size);
            font_width = font_size / 2;
            x_pos += font_width;
            str++;
        } else {
            offset = gb2312_calc_offset(str, font_size);
            lcd_show_chinese(x_pos, y, offset, back_color, font_color, font_size);
            font_width = font_size;
            x_pos += font_width;
            str += 2;
        }
        
    }
}
#endif /* USE_ASCII_EN or USE_CHINESE_EN */
