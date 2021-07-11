#include "lcd_rgb_ltdc_drv.h"

#if USE_DMA2D_EN
#include "dma2d.h"
#endif /* USE_DMA2D_EN */

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