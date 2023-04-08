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

#include "lcd_spi_drv.h"

#ifdef ESP32_PLATFORM
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#include "stm32l4xx_hal.h"
#include "spi.h"
#endif

#if USE_ASCII_FONT_LIB
#include "font.h"
#endif /* USE_ASCII_FONT_LIB */

#ifdef CONFIG_IDF_TARGET_ESP32
#define LCD_HOST    HSPI_HOST

#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22

#define PIN_NUM_DC   21
#define PIN_NUM_RST  18
#define PIN_NUM_BCKL 5
#elif defined CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define LCD_HOST    SPI2_HOST

#define PIN_NUM_MISO 37
#define PIN_NUM_MOSI 35
#define PIN_NUM_CLK  36
#define PIN_NUM_CS   45

#define PIN_NUM_DC   4
#define PIN_NUM_RST  5
#define PIN_NUM_BCKL 6
#elif defined CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2
#define LCD_HOST    SPI2_HOST

#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 7
#define PIN_NUM_CLK  6
#define PIN_NUM_CS   10
#define PIN_NUM_DC   8
#define PIN_NUM_RST  4
#define PIN_NUM_BCKL 5
#endif

#if USE_SPI_WRITE_BUF
static uint8_t write_buf[SPI_WRITE_BUFFER_SIZE];
#endif

static lcd_color_params_t s_lcd_color_params = {
    .background_color = WHITE,
    .foreground_color = BLACK
};

#ifdef ESP32_PLATFORM

static spi_device_handle_t spi;

static int lcd_cs_port(int status)
{
    // if (status) {
    //     gpio_set_level(PIN_NUM_CS, 1);
    // } else {
    //     gpio_set_level(PIN_NUM_CS, 0);
    // }
    return 0;
}

static int lcd_blk_port(int status)
{
    if (status) {
        gpio_set_level(PIN_NUM_BCKL, 1);
    } else {
        gpio_set_level(PIN_NUM_BCKL, 0);
    }
    return 0;
}

static int lcd_rst_port(int status)
{
    if (status) {
        gpio_set_level(PIN_NUM_RST, 1);
    } else {
        gpio_set_level(PIN_NUM_RST, 0);
    }
    return 0;
}

static int lcd_dc_port(int status)
{
    if (status) {
        gpio_set_level(PIN_NUM_DC, 1);
    } else {
        gpio_set_level(PIN_NUM_DC, 0);
    }
    return 0;
}

static int spi_init(void)
{
    gpio_config_t io_conf = {};
    esp_err_t ret;

    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz=10*1000*1000,
        .mode = 3,
        .spics_io_num = -1,
        .queue_size = 1,
    };

    //Initialize the lcd gpio
    io_conf.pin_bit_mask = ((1ULL<<PIN_NUM_DC) | (1ULL<<PIN_NUM_RST) | (1ULL<<PIN_NUM_BCKL));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = true;
    gpio_config(&io_conf);

    //Initialize the SPI bus
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    return 0;
}

static int spi_write_byte(uint8_t data)
{
    esp_err_t ret;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;
    ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);
    return 0;
}

static int spi_write_multi_bytes(uint8_t *data, uint16_t size)
{
    esp_err_t ret;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.length = 8 * size;
    t.tx_buffer = data;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret==ESP_OK);
    return 0;
}

void lcd_delay_ms(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

#else

static int lcd_cs_port(int status)
{
//    if (status) {
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
//    } else {
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
//    }
    return 0;
}

static int lcd_blk_port(int status)
{
    if (status) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
    return 0;
}

static int lcd_rst_port(int status)
{
    if (status) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    return 0;
}

static int lcd_dc_port(int status)
{
    if (status) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
    }
    return 0;
}



static int spi_init(void)
{
#if USE_REGISTER_METHOD
    SPI_1LINE_TX(&hspi2);
    __HAL_SPI_ENABLE(&hspi2);
#endif
    return 0;
}

static int spi_write_byte(uint8_t data)
{
#if USE_REGISTER_METHOD
    // this method on stm32l4@80Mhz(240*240), -O3, -Otime, clear time is 30 ms.
    // this method on stm32l4@80Mhz(320*240), -O3, -Otime, clear time is 55 ms.
    while(!__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_TXE));
    *((__IO uint8_t*)&hspi2.Instance->DR) = data;
#else
    // this method on stm32l4@80Mhz(240*240), -O3, -Otime, clear time is 632 ms.
    // this method on stm32l5@110Mhz(240*240), -O3, spi send with buffer, clear time is 75 ms.
    // this method on stm32l4@80Mhz(320*240), -O3, -Otime, clear time is 741 ms.
    HAL_StatusTypeDef status;
    status = HAL_SPI_Transmit(&hspi2, &data, 1, 1000);
    return status == HAL_OK ? 0 : -1;
#endif
}

static int spi_write_multi_bytes(uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef status;
    status = HAL_SPI_Transmit(&hspi2, data, size, 1000);
    return status == HAL_OK ? 0 : -1;
}

void lcd_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

#endif

static lcd_spi_drv_t lcd_spi_drv = {
    .cs  = lcd_cs_port,
    .blk = lcd_blk_port,
    .rst = lcd_rst_port,
    .dc  = lcd_dc_port,

    .init = spi_init,
    .write_byte = spi_write_byte,
    .write_multi_bytes = spi_write_multi_bytes,

    .delay = lcd_delay_ms,
};
#define lcd (&lcd_spi_drv)

static void lcd_hard_reset(void)
{
    lcd->rst(0);
    lcd->delay(100);
    lcd->rst(1);
}

static void lcd_write_cmd(uint8_t cmd)
{
    lcd->dc(0);
    lcd->write_byte(cmd);
}

static void lcd_write_data(uint8_t dat)
{
    lcd->dc(1);
    lcd->write_byte(dat);
}

static void lcd_write_color(const uint16_t color)
{
#if USE_REGISTER_METHOD
    lcd->write_byte(color >> 8);
    lcd->write_byte(color);
#else
    uint8_t data[2];

    data[0] = color >> 8;
    data[1] = color;

    lcd->write_multi_bytes(data, 2);
#endif
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
    lcd->cs(0);
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
    lcd->cs(1);
}

void lcd_display_on(void)
{
    lcd->blk(1);
}

void lcd_display_off(void)
{
    lcd->blk(0);
}

void lcd_color_set(uint16_t back_color, uint16_t fore_color)
{
    s_lcd_color_params.background_color = back_color;
    s_lcd_color_params.foreground_color = fore_color;
}


void lcd_init(void)
{
    /* GPIO initialization code in main.c */

    /* SPI initialization code in main.c */

    lcd->init();

    /* LCD Hard Reset */
    lcd_hard_reset();
    lcd->delay(120);

    /* Sleep Out */
    lcd_write_cmd(0x11);

    /* wait for power stability */
    lcd->delay(120);

    /* Memory Data Access Control */
    lcd_write_cmd(0x36);
#if LCD_DIRECTION == 0
    lcd_write_data(0x00);
#elif LCD_DIRECTION == 1
    lcd_write_data(0x60);
#elif LCD_DIRECTION == 2
    lcd_write_data(0xA0);
#elif LCD_DIRECTION == 3
    lcd_write_data(0xC0);
#else
#error "lcd screen direction error!"
#endif

    /* RGB 5-6-5-bit  */
    lcd_write_cmd(0x3A);
    lcd_write_data(0x05);

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
    lcd_write_data(0x01);	//111MHZ

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

#if USE_DISPLAY_INVERSION
    /* Display Inversion On */
    lcd_write_cmd(0x21);
    lcd_write_cmd(0x29);
#else
    /* Display Inversion Off */
    lcd_write_cmd(0x20);
    lcd_write_cmd(0x29);
#endif

    lcd_clear();

    lcd_display_on();
}

void lcd_draw_point(uint16_t x, uint16_t y,uint16_t color)
{
    lcd_address_set(x, y, x, y);
    lcd->cs(0);
    lcd->dc(1);
    lcd_write_color(color);
    lcd->cs(1);
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
        lcd->cs(0);
        lcd->dc(1);
        for (i = 0; i < x2 - x1; i++) {
            lcd_write_color(color);
        }
        lcd->cs(1);
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
#if USE_SPI_WRITE_BUF
    uint16_t i, j;
    uint8_t data[2] = {0};
    uint32_t size, remain_size;

    size = (x2 - x1 + 1) * (y2 - y1 + 1) * 2;

    data[0] = color >> 8;
    data[1] = color;

    for (j = 0; j < SPI_WRITE_BUFFER_SIZE / 2; j++) {
        write_buf[j * 2] =  data[0];
        write_buf[j * 2 + 1] =  data[1];
    }

    lcd_address_set(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    lcd->cs(0);
    lcd->dc(1);
    for (i = 0; i < (size / SPI_WRITE_BUFFER_SIZE); i++) {
        lcd->write_multi_bytes(write_buf, SPI_WRITE_BUFFER_SIZE);
    }
    remain_size = size % SPI_WRITE_BUFFER_SIZE;
    if (remain_size) {
        lcd->write_multi_bytes(write_buf, SPI_WRITE_BUFFER_SIZE);
    }
    lcd->cs(1);
#else
    uint32_t size, i;

    size = (x2 - x1 + 1) * (y2 - y1 + 1) * 2;

    lcd_address_set(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    lcd->cs(0);
    lcd->dc(1);
    for (i = 0; i < size; i++) {
        lcd_write_color(color);
    }
    lcd->cs(1);
#endif
}

void lcd_fill_with_buffer(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color)
{
    uint32_t size, i;

    size = (x2 - x1 + 1) * (y2 - y1 + 1);

    lcd_address_set(x1, y1, x2, y2);
    lcd->cs(0);
    lcd->dc(1);
    for (i = 0; i < size; i++) {
        lcd_write_color(*color++);
    }
    lcd->cs(1);
}

void lcd_clear(void)
{
    uint16_t back_color = s_lcd_color_params.background_color;

    lcd_fill_rect(0, 0, LCD_WIDTH -1, LCD_HEIGHT - 1, back_color);
}

void lcd_clear_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint16_t back_color = s_lcd_color_params.background_color;

    lcd_fill_rect(x1, y1, x2, y2, back_color);
}

void lcd_draw_char(uint16_t x, uint16_t y, char ch, uint8_t font_size)
{
    uint16_t i, j;
    uint16_t font_total_bytes, font_width, font_height;
    uint8_t *font_ptr;
    uint8_t bit_width, temp;
    uint16_t back_color, fore_color;

    if((x > (LCD_WIDTH - font_size / 2)) || (y > (LCD_HEIGHT - font_size)))	{
        return;
    }

    font_width = font_size / 2;
    font_height = font_size;
    font_total_bytes = (font_width / 8 + ((font_width % 8) ? 1 : 0)) * font_height;
    ch = ch - ' ';
    back_color = s_lcd_color_params.background_color;
    fore_color = s_lcd_color_params.foreground_color;

    switch (font_size) {
        case 12:
            bit_width = 6;
            font_ptr = (uint8_t*)&asc2_1206[0] + ch * 12;
        case 16:
            bit_width = 8;
            font_ptr = (uint8_t*)&asc2_1608[0] + ch * 16;
            break;
        case 24:
            font_ptr = (uint8_t*)&asc2_2412[0] + ch * 24;
            break;
        case 32:
            bit_width = 8;
            font_ptr = (uint8_t*)&asc2_3216[0] + ch * 32;
            break;
        default:
            return;
    }
    lcd_address_set(x, y, x+font_width-1, y+font_height-1);
    lcd->cs(0);
    lcd->dc(1);
    for (i = 0; i < font_total_bytes; i++) {
        temp = *(font_ptr + i);
        if (font_size == 24) {
            bit_width = (i % 2 == 0) ? 8 : 4;
        }
        for (j = 0; j < bit_width; j++) {
            if(temp & 0x80){
                lcd_write_color(fore_color);
            } else {
                lcd_write_color(back_color);

            }
            temp <<= 1;
        }
    }
    lcd->cs(1);
}

void lcd_draw_chinese_char(uint16_t x, uint16_t y, uint8_t font_width, uint8_t font_height, uint8_t *font_data)
{
    uint16_t i, j;
    uint8_t bit_width, temp;
    uint16_t font_total_bytes;
    uint16_t back_color, fore_color;

    if ((x > (LCD_WIDTH - font_width)) || (y > (LCD_HEIGHT - font_height)))	{
        return;
    }

    bit_width = 8;
    back_color = s_lcd_color_params.background_color;
    fore_color = s_lcd_color_params.foreground_color;
    font_total_bytes = (font_width / 8 + ((font_width % 8) ? 1 : 0)) * font_height;

    lcd_address_set(x, y, x + font_width - 1, y + font_height - 1);
    lcd->cs(0);
    lcd->dc(1);
    for (i = 0; i < font_total_bytes; i++) {
        temp = *(font_data + i);
        for (j = 0; j < bit_width; j++) {
            if(temp & 0x80){
                lcd_write_color(fore_color);
            } else {
                lcd_write_color(back_color);
            }
            temp <<= 1;
        }
    }
    lcd->cs(1);
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
    lcd->cs(0);
    lcd->dc(1);
    for (i = 0;i <= img_size / 65536; i++) {
        if (remain_size / 65536 >= 1) {
            lcd->write_multi_bytes((uint8_t *)p, 65535);
            p += 65535;
            remain_size -= 65535;
        } else {
            lcd->write_multi_bytes((uint8_t *)p, remain_size % 65535);
        }
    }
    lcd->cs(1);
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
