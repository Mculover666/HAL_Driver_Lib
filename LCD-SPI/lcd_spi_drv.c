/**
 * @Copyright 			(c) 2019,mculover666 All rights reserved	
 * @filename  			lcd_spi_drv.h
 * @breif				Drive ST7789 LCD based on spi interface
 * @changelog
 *            			v1.0    finish baic function                mculover666    2019/7/10
 *                      v2.0    add macro define to control build   mculover666    2019/7/13
 *                      v2.1    add support for scroll function     mculover666    2021/5/19
 */

#include "lcd_spi_drv.h"

#if USE_ASCII_FONT_LIB
#include "font.h"
#endif /* USE_ASCII_FONT_LIB */

#define LCD_TOTAL_BUF_SIZE	(LCD_Width*LCD_Height*2)
#define LCD_BUFFER_SIZE 1152

static uint8_t lcd_buf[LCD_BUFFER_SIZE];

/**
 * @brief	LCD hard reset
 * @param   none
 * @return  none
 */
static void lcd_hard_reset(void)
{
    LCD_PWR(0);
    LCD_RST(0);
    HAL_Delay(100);
    LCD_RST(1);
}

/**
 * @brief	Write bytes to LCD
 * @param   data    pointer of data buffer to send
 * @param   size    length of data buffer to send
 * @return  none
 */
static void spi_write_bytes(uint8_t *data, uint16_t size)
{
    HAL_SPI_Transmit(&LCD_SPI_HANDLER, data, size, 1000);
}

/**
 * @brief	Write command to lcd.
 * @param   cmd    lcd control command
 * @return  none
 */
static void lcd_write_cmd(uint8_t cmd)
{
    LCD_WR_RS(0);
    spi_write_bytes(&cmd, 1);
}

/**
 * @brief	Write data to lcd.
 * @param 	dat    lcd parameter/RAM data
 * @return  none
 */
static void lcd_write_data(uint8_t dat)
{
    LCD_WR_RS(1);
    spi_write_bytes(&dat, 1);
}
/**
 * @brief	write color data to LCD.
 * @param   color    the color data  
 * @return  none
 * @note    color data format is RGB 565, 16bit.
 */
static void lcd_write_color(const uint16_t color)
{
    uint8_t data[2] = {0};

    data[0] = color >> 8;
    data[1] = color;

    LCD_WR_RS(1);
    spi_write_bytes(data, 2);
}

/**
 * @brief   Convert the RGB 565 color to hex.
 * @param   r   red value of color(0 - 63)
 * @param   g   green value of color(0 - 127)
 * @param   b   GREEN value of color(0 - 63)
 * @return  hex value of color
 * @note    color format: rgb 565
 * 
*/
uint16_t rgb2hex_565(uint16_t r, uint16_t g, uint16_t b)
{
    uint16_t color;
    
    r = (r & 0x1F) << 11;
    g = (g & 0x3F) << 5;
    b = b & 0x1F;
    color = r | g | b;

    return color;
}

/**
 * @brief	set lcd display address
 * @param   x1,y2   start address
 * @param   x2,y2	end address
 * @return  none
 */
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

/**
 * @breif	turn on LCD power.
 * @param   none
 * @return  none
 */
void lcd_display_on(void)
{
    LCD_PWR(1);
}

/**
 * @brief	turn off LCD power.
 * @param   none
 * @return  none
 */
void lcd_display_off(void)
{
    LCD_PWR(0);
}

/**
 * @brief	clear LCD with color.
 * @param   color   LCD color
 * @return  none
 */
void lcd_clear(uint16_t color)
{
    uint16_t i, j;
    uint8_t data[2] = {0};  //LCD?????????16bit??data[0]???????????��??data[1]???????????��
    uint16_t remain_size;

    data[0] = color >> 8;
    data[1] = color;

    lcd_address_set(0, 0, LCD_Width - 1, LCD_Height - 1);

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
    uint8_t data;
    
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

/**
 * @brief	LCD?????
 * @param   none
 * @return  none
 */
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

    lcd_address_set(0, 0, LCD_Width - 1, LCD_Height - 1);
    lcd_clear(DEFAULT_COLOR_AFTER_INIT);

    LCD_PWR(1);
}

/**
 * @brief   draw a point with color.
 * @param   x,y     point address
 * @param   color   point color
 * @return  none
 */
void lcd_draw_color_point(uint16_t x, uint16_t y,uint16_t color)
{
    lcd_address_set(x, y, x, y);
    lcd_write_color(color);
}

/**
 * @brief   draw a line with color.
 * @param   x1,y1   line start address
 * @param   x2,y2   line end address
 * @param   color   line color
 * @return  none
 */
void lcd_draw_color_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t	i = 0;
    int16_t		delta_x = 0, delta_y = 0;
    int8_t		incx = 0, incy = 0;
    uint16_t	distance = 0;
    uint16_t    t = 0;
    uint16_t	x = 0, y = 0;
    uint16_t 	x_temp = 0, y_temp = 0;
	

    if (y1 == y2) {
        lcd_address_set(x1, y1, x2, y2);

        for(i = 0; i < x2 - x1; i++) {
            lcd_buf[2 * i] = color >> 8;
            lcd_buf[2 * i + 1] = color;
        }

        LCD_WR_RS(1);
        spi_write_bytes(lcd_buf, (x2 - x1) * 2);
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
            lcd_draw_color_point(x, y, color);
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

/**
 * @brief   draw a rect with color.
 * @param   x1,y1   rect start address
 * @param   x2,y2   rect end address
 * @param   color   rect color
 * @return  none
 */
void lcd_draw_color_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    lcd_draw_color_line(x1, y1, x2, y1, color);
    lcd_draw_color_line(x1, y1, x1, y2, color);
    lcd_draw_color_line(x1, y2, x2, y2, color);
    lcd_draw_color_line(x2, y1, x2, y2, color);
}

/**
 * @brief   draw a circle with color.
 * @param   x,y     center of circle address
 * @param   r       radius of cirlce
 * @param   color   circle color
 * @return  none
 */
void lcd_draw_color_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
	int16_t a = 0, b = r;
    int16_t d = 3 - (r << 1);
		
    if (x - r < 0 || x + r > LCD_Width || y - r < 0 || y + r > LCD_Height) {
		return;
    }
		
    while(a <= b) {
        lcd_draw_color_point(x - b, y - a, color);
        lcd_draw_color_point(x + b, y - a, color);
        lcd_draw_color_point(x - a, y + b, color);
        lcd_draw_color_point(x - b, y - a, color);
        lcd_draw_color_point(x - a, y - b, color);
        lcd_draw_color_point(x + b, y + a, color);
        lcd_draw_color_point(x + a, y - b, color);
        lcd_draw_color_point(x + a, y + b, color);
        lcd_draw_color_point(x - b, y + a, color);
        a++;

        if (d < 0) {
			d += 4 * a + 6;
        } else {
            d += 10 + 4 * (a - b);
            b--;
        }

        lcd_draw_color_point(x + a, y + b, color);
    }
}

/**
 * @brief   fill a rect with color.
 * @param   x1,y1   rect start address
 * @param   x2,y2   rect end address
 * @param   color   rect color
 * @return  none
 */
void lcd_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t i = 0;
    uint32_t size = 0, size_remain = 0;

    size = (x2 - x1 + 1) * (y2 - y1 + 1) * 2;

    if (size > LCD_BUFFER_SIZE) {
        size_remain = size - LCD_BUFFER_SIZE;
        size = LCD_BUFFER_SIZE;
    }

    lcd_address_set(x1, y1, x2, y2);

    while (1) {
        for (i = 0; i < size / 2; i++) {
            lcd_buf[2 * i] = color >> 8;
            lcd_buf[2 * i + 1] = color;
        }

        LCD_WR_RS(1);
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
}

#if USE_ASCII_FONT_LIB
/**
 * @brief   display a character with color.
 * @param   x1,y1       character start address
 * @param   ch          character
 * @param   back_color  background color
 * @param   font_color  front color
 * @param   font_size   font size of character
 * @return  none
 */
void lcd_show_char(uint16_t x, uint16_t y, char ch, uint16_t back_color, uint16_t font_color, uint8_t font_size)
{
	int i = 0, j = 0;
	uint8_t temp = 0;
	uint8_t size = 0;
	uint8_t t = 0;
	
	 if ((x > (LCD_Width - font_size / 2)) || (y > (LCD_Height - font_size))) {	
		 return;
     }
	
	 lcd_address_set(x, y, x + font_size/2 - 1, y + font_size - 1);
	 
	 ch = ch - ' ';
	 
	 if ((font_size == 16) || (font_size == 32)) {
        size = (font_size / 8 + ((font_size % 8) ? 1 : 0)) * (font_size / 2);

        for (i = 0; i < size; i++) {
            if(font_size == 16) {
                temp = asc2_1608[ch][i];
            } else if (font_size == 32) {
                temp = asc2_3216[ch][i];
            } else {
                return;
            }

            for (j = 0; j < 8; j++) {
                if(temp & 0x80) {
                    lcd_write_color(font_color);
                } else { 
                    lcd_write_color(back_color);
                }
                temp <<= 1;
            }
        }
	 } else if(font_size == 12) {
        size = (font_size / 8 + ((font_size % 8) ? 1 : 0)) * (font_size / 2);

        for (i = 0; i < size; i++) {
            temp = asc2_1206[ch][i];
            for (j = 0; j < 6; j++) {
                if (temp & 0x80) {
                    lcd_write_color(font_color);
                } else { 
                    lcd_write_color(back_color);
                }
                temp <<= 1;
            }
        }
	 } else if (font_size == 24) {
        size = (font_size * 16) / 8;

        for (i = 0; i < size; i++) {
            temp = asc2_2412[ch][i];
            if (i % 2 == 0) {
                t = 8;
            } else {
                t = 4;
            }
            
            for (j = 0; j < t; j++) {
                if (temp & 0x80) {
                    lcd_write_color(font_color);
                } else { 
                    lcd_write_color(back_color);
                }
                temp <<= 1;
            }
        }
    } else {
		 return;
    }
}

/**
 * @brief   display a string with color.
 * @param   x,y         string start address
 * @param   str         string
 * @param   back_color  background color
 * @param   font_color  front color
 * @param   font_size   font size of character
 * @return  none
 */
void lcd_show_str(uint16_t x, uint16_t y, char* str, uint16_t back_color, uint16_t font_color, uint8_t font_size)
{
	while ((*str <= '~') && (*str >= ' ')) {
        lcd_show_char(x,y,*str,back_color, font_color,font_size);
        x += font_size / 2;
        str++;
	}
}
#endif /* USE_ASCII_FONT_LIB */

/**
 * @brief   display a six point star with color.
 * @param   x,y         star start address
 * @param   r           raduis of star
 * @param   back_color  star color
 * @return  none
 */
void lcd_draw_color_sixpointstar(uint16_t x, uint16_t y, uint8_t r, uint16_t color)
{
		uint16_t a = r / 2;
		uint16_t b = 1.432*r;
	
		lcd_draw_color_line(x-b,y-a,x+b,y-a,color);
		lcd_draw_color_line(x+b,y-a,x,y+r,color);
		lcd_draw_color_line(x,y+r,x-b,y-a,color);
	
		lcd_draw_color_line(x-b,y+a,x+b,y+a,color);
		lcd_draw_color_line(x+b,y+a,x,y-r,color);
		lcd_draw_color_line(x,y-r,x-b,y+a,color);

}

#if USE_PICTURE_DISPLAY
/**
 * @brief   display a picture.
 * @param   x,y         picture start address
 * @param   width       width of picture
 * @param   height      height of pictue
 * @param   p           pointer to picture buffer
 * @return  none
 */
void lcd_show_image(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *p)
{
	uint32_t img_size = width * height * 2;
	uint32_t remain_size = img_size;
	uint8_t i = 0;
	
    if(x + width > LCD_Width || y + height > LCD_Height) {
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
#endif /*  USE_PICTURE_DISPLAY */
