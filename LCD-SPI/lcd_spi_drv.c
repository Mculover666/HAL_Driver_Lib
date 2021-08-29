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

#include "lcd_spi_drv.h"

#if USE_ASCII_FONT_LIB
#include "font.h"
#endif /* USE_ASCII_FONT_LIB */

static lcd_color_params_t s_lcd_color_params = {
    .background_color = BLACK,
    .foreground_color = WHITE
};

static void lcd_hard_reset(void)
{
    LCD_PWR(0);
    LCD_RST(0);
    HAL_Delay(100);
    LCD_RST(1);
}

static void spi_write_bytes(uint8_t *data, uint16_t size)
{
    HAL_SPI_Transmit(&LCD_SPI_HANDLER, data, size, 1000);
}

static void lcd_write_cmd(uint8_t cmd)
{
    LCD_WR_RS(0);
    spi_write_bytes(&cmd, 1);
}

static void lcd_write_data(uint8_t dat)
{
    LCD_WR_RS(1);
    spi_write_bytes(&dat, 1);
}

static void lcd_write_color(const uint16_t color)
{
    uint8_t data[2] = {0};

    data[0] = color >> 8;
    data[1] = color;

    LCD_WR_RS(1);
    spi_write_bytes(data, 2);
}

uint16_t rgb2hex_565(uint16_t r, uint16_t g, uint16_t b)
{
    uint16_t color;
    
    r = (r & 0x1F) << 11;
    g = (g & 0x3F) << 5;
    b = b & 0x1F;
    color = r | g | b;

    return color;
}

static void lcd_address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    lcd_write_cmd(0x2a);
    lcd_write_data(x1 >> 8);
    lcd_write_data(x1);
    lcd_write_data(x2 >> 8);
    lcd_write_data(x2);

    lcd_write_cmd(0x2b);
    lcd_write_data(y1 >> 8);
    lcd_write_data(y1);
    lcd_write_data(y2 >> 8);
    lcd_write_data(y2);

    lcd_write_cmd(0x2C);
}

void lcd_display_on(void)
{
    LCD_PWR(1);
}

void lcd_display_off(void)
{
    LCD_PWR(0);
}

void lcd_color_set(uint16_t back_color, uint16_t fore_color)
{
    s_lcd_color_params.background_color = back_color;
    s_lcd_color_params.foreground_color = fore_color;
}

void lcd_clear(void)
{
#if 0
    uint16_t i, j;
    uint8_t data[2] = {0};
    uint16_t remain_size;
    uint16_t color = s_lcd_color_params.background_color;

    data[0] = color >> 8;
    data[1] = color;

    lcd_address_set(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    for(j = 0; j < LCD_BUFFER_SIZE / 2; j++)
    {
        lcd_buf[j * 2] =  data[0];
        lcd_buf[j * 2 + 1] =  data[1];
    }

    LCD_WR_RS(1);

    for(i = 0; i < (LCD_TOTAL_BUF_SIZE / LCD_BUFFER_SIZE); i++)
    {
        spi_write_bytes(lcd_buf, LCD_BUFFER_SIZE);
    }
    
    remain_size = LCD_TOTAL_BUF_SIZE % LCD_BUFFER_SIZE;
    if (remain_size) {
        spi_write_bytes(lcd_buf, remain_size);
    }
#else
    uint32_t size, i;
    uint16_t color = s_lcd_color_params.background_color;

    size = LCD_WIDTH * LCD_HEIGHT;
    
    lcd_address_set(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    for (i = 0; i < size; i++) {
        lcd_write_color(color);
    }
#endif
}

void lcd_init(void)
{
    /* GPIO initialization code in main.c */

    /* SPI initialization code in main.c */

    /* LCD Hard Reset */
    lcd_hard_reset();
    HAL_Delay(120);
	
    /* Sleep Out */
    lcd_write_cmd(0x11);

    /* wait for power stability */
    HAL_Delay(120);

    /* Memory Data Access Control */
    lcd_write_cmd(0x36);
    lcd_write_data(0x00);

    /* RGB 5-6-5-bit  */
    lcd_write_cmd(0x3A);
    lcd_write_data(0x65);

    /* Porch Setting */
    lcd_write_cmd(0xB2);
    lcd_write_data(0x0C);
    lcd_write_data(0x0C);
    lcd_write_data(0x00);
    lcd_write_data(0x33);
    lcd_write_data(0x33);

    /*  Gate Control */
    lcd_write_cmd(0xB7);
    lcd_write_data(0x72);

    /* VCOM Setting */
    lcd_write_cmd(0xBB);
    lcd_write_data(0x3D);   //Vcom=1.625V

    /* LCM Control */
    lcd_write_cmd(0xC0);
    lcd_write_data(0x2C);

    /* VDV and VRH Command Enable */
    lcd_write_cmd(0xC2);
    lcd_write_data(0x01);

    /* VRH Set */
    lcd_write_cmd(0xC3);
    lcd_write_data(0x19);

    /* VDV Set */
    lcd_write_cmd(0xC4);
    lcd_write_data(0x20);

    /* Frame Rate Control in Normal Mode */
    lcd_write_cmd(0xC6);
    lcd_write_data(0x0F);	//60MHZ

    /* Power Control 1 */
    lcd_write_cmd(0xD0);
    lcd_write_data(0xA4);
    lcd_write_data(0xA1);

    /* Positive Voltage Gamma Control */
    lcd_write_cmd(0xE0);
    lcd_write_data(0xD0);
    lcd_write_data(0x04);
    lcd_write_data(0x0D);
    lcd_write_data(0x11);
    lcd_write_data(0x13);
    lcd_write_data(0x2B);
    lcd_write_data(0x3F);
    lcd_write_data(0x54);
    lcd_write_data(0x4C);
    lcd_write_data(0x18);
    lcd_write_data(0x0D);
    lcd_write_data(0x0B);
    lcd_write_data(0x1F);
    lcd_write_data(0x23);

    /* Negative Voltage Gamma Control */
    lcd_write_cmd(0xE1);
    lcd_write_data(0xD0);
    lcd_write_data(0x04);
    lcd_write_data(0x0C);
    lcd_write_data(0x11);
    lcd_write_data(0x13);
    lcd_write_data(0x2C);
    lcd_write_data(0x3F);
    lcd_write_data(0x44);
    lcd_write_data(0x51);
    lcd_write_data(0x2F);
    lcd_write_data(0x1F);
    lcd_write_data(0x1F);
    lcd_write_data(0x20);
    lcd_write_data(0x23);
    
#if USE_VERTICAL_SCROLL
    /* Defign Scroll Area */
    lcd_set_scroll_area(0, 240, 80);
#endif

    /* Display Inversion On */
    lcd_write_cmd(0x21);
    lcd_write_cmd(0x29);
    
    lcd_clear();

    LCD_PWR(1);
}

void lcd_draw_point(uint16_t x, uint16_t y,uint16_t color)
{
    lcd_address_set(x, y, x, y);
    lcd_write_color(color);
}

void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t	i = 0;
    int16_t		delta_x = 0, delta_y = 0;
    int8_t		incx = 0, incy = 0;
    uint16_t	distance = 0;
    uint16_t    t = 0;
    uint16_t	x = 0, y = 0;
    uint16_t 	x_temp = 0, y_temp = 0;
	
    if (y1 == y2) {
        lcd_address_set(x1, y1, x2 - 1, y2);
        for (i = 0; i < x2 - x1; i++) {
            lcd_write_color(color);
        }
        return;
    } else {
        delta_x = x2 - x1;
        delta_y = y2 - y1;
        if (delta_x > 0) {
            incx = 1;
        } else if (delta_x == 0) {
            incx = 0;
        } else {
            incx = -1;
            delta_x = -delta_x;
        }
        
        if (delta_y > 0) {
            incy = 1;
        } else if (delta_y == 0) {
            incy = 0;
        } else {
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
        for (t = 0; t <= distance + 1;t++) {
            lcd_draw_point(x, y, color);
            x_temp += delta_x;	
            if (x_temp > distance) {
                x_temp -= distance;		
                x += incx;
            }
            y_temp += delta_y;
            if (y_temp > distance) {
                y_temp -= distance;
                y += incy;
            }
        }
    }
}

void lcd_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    if (y1 == y2) {
        lcd_draw_line(x1, y1, x2, y1, color);
    } else {
        lcd_draw_line(x1, y1, x2, y1, color);
        lcd_draw_line(x1, y1, x1, y2, color);
        lcd_draw_line(x1, y2, x2, y2, color);
        lcd_draw_line(x2, y1, x2, y2, color);
    }
}

void lcd_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
	int16_t a = 0, b = r;
    int16_t d = 3 - (r << 1);
		
    if (x - r < 0 || x + r > LCD_WIDTH || y - r < 0 || y + r > LCD_HEIGHT) {
		return;
    }
		
    while(a <= b) {
        lcd_draw_point(x - b, y - a, color);
        lcd_draw_point(x + b, y - a, color);
        lcd_draw_point(x - a, y + b, color);
        lcd_draw_point(x - b, y - a, color);
        lcd_draw_point(x - a, y - b, color);
        lcd_draw_point(x + b, y + a, color);
        lcd_draw_point(x + a, y - b, color);
        lcd_draw_point(x + a, y + b, color);
        lcd_draw_point(x - b, y + a, color);
        a++;

        if (d < 0) {
			d += 4 * a + 6;
        } else {
            d += 10 + 4 * (a - b);
            b--;
        }

        lcd_draw_point(x + a, y + b, color);
    }
}

void lcd_fill_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
#if 0
    uint16_t i = 0;
    uint32_t size = 0, size_remain = 0;

    size = (x2 - x1 + 1) * (y2 - y1 + 1) * 2;

    if (size > LCD_BUFFER_SIZE) {
        size_remain = size - LCD_BUFFER_SIZE;
        size = LCD_BUFFER_SIZE;
    }

    lcd_address_set(x1, y1, x2, y2);
    LCD_WR_RS(1);

    while (1) {
        for (i = 0; i < size / 2; i++) {
            lcd_buf[2 * i] = color >> 8;
            lcd_buf[2 * i + 1] = color;
        }
        
        spi_write_bytes(lcd_buf, size);

        if (size_remain == 0) {
            break;
        }

        if (size_remain > LCD_BUFFER_SIZE) {
            size_remain = size_remain - LCD_BUFFER_SIZE;
        } else {
            size = size_remain;
            size_remain = 0;
        }
    }
#else
    uint32_t size, i;

    size = (x2 - x1) * (y2 - y1);
    
    lcd_address_set(x1, y1, x2 - 1, y2 - 1);
    for (i = 0; i < size; i++) {
        lcd_write_color(color);
    }
#endif
}

void lcd_clear_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint16_t back_color = s_lcd_color_params.background_color;
    
    lcd_fill_rect(x1, y1, x2, y2, back_color);
}

#include <stdio.h>

void lcd_draw_char(uint16_t x, uint16_t y, char ch, uint8_t font_size)
{
    uint16_t i, j;
    uint16_t x_pos, y_pos, size, font_width, font_height;
    uint8_t *font_ptr;
    uint8_t bit_width, temp;
    uint16_t back_color, fore_color;
	
    if((x > (LCD_WIDTH - font_size / 2)) || (y > (LCD_HEIGHT - font_size)))	{
        return;
    }
	
    font_width = font_size / 2;
    font_height = font_size;
    size = (font_width / 8 + ((font_width % 8) ? 1 : 0)) * font_height;
    x_pos = x;
    y_pos = y;
    ch = ch - ' ';
    back_color = s_lcd_color_params.background_color;
    fore_color = s_lcd_color_params.foreground_color;

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
    lcd_address_set(x, y, x+font_width-1, y+font_height-1);
    for (i = 0; i < size; i++) {
        temp = *(font_ptr + i);
        if (font_size == 24) {
            bit_width = (i % 2 == 0) ? 8 : 4;
        }
        //lcd_address_set(x_pos, y_pos, x_pos+8, y_pos);
        for (j = 0; j < bit_width; j++) {
            if(temp & 0x80){
                //lcd_draw_point(x_pos, y_pos, fore_color);
                lcd_write_color(fore_color);
            } else {
                //lcd_draw_point(x_pos, y_pos, back_color);
                lcd_write_color(back_color);
               
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

const unsigned char hz_16x16[][32] = {
    
{0x01,0x00,0x01,0x00,0xFF,0xFE,0x01,0x00,0x01,0x00,0x7F,0xFC,0x48,0x24,0x44,0x44,// до //
 0x4F,0xE4,0x41,0x04,0x41,0x04,0x5F,0xF4,0x41,0x04,0x41,0x04,0x41,0x14,0x40,0x08},


{0x00,0x40,0x20,0x40,0x10,0x40,0x10,0x40,0x87,0xFC,0x44,0x44,0x44,0x44,0x14,0x44,// см //
 0x14,0x44,0x27,0xFC,0xE4,0x44,0x24,0x44,0x24,0x44,0x24,0x44,0x27,0xFC,0x04,0x04},

};

#include <string.h>

static int font_get_data(uint32_t offset, uint8_t font_size, uint8_t *buffer, uint16_t len)
{
    if (!buffer || !len) {
        return -1;
    }
    
    memcpy(buffer, &hz_16x16[offset], len);
    
    return 0;
}

void lcd_draw_chinese(uint16_t x, uint16_t y, uint32_t offset, uint8_t font_size)
{
    uint16_t i, j;
    uint16_t x_pos, y_pos;
    uint8_t bit_width, temp;
    uint8_t buffer[CHINESE_FONT_BUF_MAX_SIZE_ONE_CHR];
    uint16_t font_total_bytes, font_read_bytes;
    uint16_t back_color, fore_color;
	
    if ((x > (LCD_WIDTH - font_size)) || (y > (LCD_HEIGHT - font_size)))	{
        return;
    }
	
    x_pos = x;
    y_pos = y;
    bit_width = 8;
    back_color = s_lcd_color_params.background_color;
    fore_color = s_lcd_color_params.foreground_color;
    font_total_bytes = (font_size / 8 + ((font_size % 8) ? 1 : 0)) * font_size;

    // so, when you want to show big chinese char, 
    // you should incrase the CHINESE_FONT_BUF_MAX_SIZE_ONE_CHR
    // to adjust buffer size(>= font_total_bytes).
    
#define MIN(a, b) (a < b ? a : b)
    font_read_bytes = MIN(sizeof(buffer), font_total_bytes);
    
    if (font_get_data(offset, font_size, buffer, font_read_bytes) != 0) {
        return;
    }

    for (i = 0; i < font_read_bytes; i++) {
        temp = buffer[i];
        for (j = 0; j < bit_width; j++) {
            if(temp & 0x80){
                lcd_draw_point(x_pos, y_pos, fore_color);
            } else {
                lcd_draw_point(x_pos, y_pos, back_color);
            }
            temp <<= 1;
            x_pos++;
        }
        if (x_pos >= (x + font_size)) {
            y_pos++;
            x_pos = x;
        }
    }
}

void lcd_draw_text(uint16_t x, uint16_t y, char* str, uint8_t font_size)
{
	while ((*str <= '~') && (*str >= ' ')) {
        lcd_draw_char(x, y, *str, font_size);
        x += font_size / 2;
        str++;
	}
}

void lcd_show_image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *p)
{
	uint32_t img_size = width * height * 2;
	uint32_t remain_size = img_size;
	uint8_t i = 0;
	
    if(x + width > LCD_WIDTH || y + height > LCD_HEIGHT) {
        return;
    }
				
    lcd_address_set(x, y, x + width - 1, y + height - 1);

    LCD_WR_RS(1);

    for (i = 0;i <= img_size / 65536; i++) {
        if (remain_size / 65536 >= 1) {
            spi_write_bytes((uint8_t *)p, 65535);
            p += 65535;
            remain_size -= 65535;
        } else {
            spi_write_bytes((uint8_t *)p, remain_size % 65535);
        }
    }  
}

#if USE_VERTICAL_SCROLL

/**
 * @brief	Vertical Scrolling Definition.
 * @param   tfa    top fixed area
 * @param   vsa    scroll area
 * @param   bta    bottom fixed area
 * @return  errcode
 * @retval  0      success
 * @retval  -1     fail 
 */
int lcd_set_scroll_area(uint16_t tfa, uint16_t vsa, uint16_t bta)
{
    if (tfa + vsa + bta != 320) {
        return -1;
    }
    
    lcd_write_cmd(0x33);
    lcd_write_data(tfa >> 8);
    lcd_write_data(tfa);
    lcd_write_data(vsa >> 8);
    lcd_write_data(vsa);    
    lcd_write_data(bta >> 8);
    lcd_write_data(bta);
    
    return 0;
}

/**
 * @brief	Set Vertical scroll start address of RAM.
 * @param   vsp    scroll start address of RAM
 * @return  none
 */
void lcd_set_scroll_start_address(uint16_t vsp)
{
    lcd_write_cmd(0x37);
    lcd_write_data(vsp>>8);
    lcd_write_data(vsp);
}
#endif /* USE_VERTICAL_SCROLL */
