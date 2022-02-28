/******************************************************
 * @file    lcd_fsmc.h
 * @brief   使用FSMC外设驱动TFT-LCD屏幕（MCU屏）
 * @author  Mculover666
 * @date    2020/08/27
 * @note    
 *          目前支持的驱动器：
 *            - ILI9341(0x9341)
 *            - NT35310(0x5310)
 *            - NT35510(0x5510)
******************************************************/

#ifndef _LCD_FSMC_H_
#define _LCD_FSMC_H_

#include "fsmc.h"

/**
 * @brief    LCD屏幕参数
 * @param    lcd_width     LCD屏幕宽度
 * @param    lcd_height    LCD屏幕高度
 * @param    lcd_id        LCD 驱动IC ID
 * @param    lcd_direction LCD横屏显示还是竖屏显示，0-竖屏，1-横屏
 * @param    wram_cmd      开始写gram指令
 * @param    set_x_cmd     设置x坐标指令
 * @param    set_y_cmd     设置y坐标指令
*/
typedef struct lcd_params_st {
    uint16_t lcd_width;
    uint16_t lcd_height;
    uint16_t lcd_id;
    uint8_t  lcd_direction;
    uint16_t wram_cmd;
    uint16_t set_x_cmd;
    uint16_t set_y_cmd;
} lcd_params_t;

/**
 * @brief    LCD背光状态
 * @param    LCD_BACKLIGHT_OFF 关闭背光
 * @param    LCD_BACKLIGHT_ON  打开背光
*/
typedef enum lcd_backlight_state_en {
    LCD_BACKLIGHT_OFF = 0,
    LCD_BACKLIGHT_ON  = 1,
} lcd_backlight_state_t;

/**
 * @brief    LCD扫描方向
 * @note     L-左，R-右，U-上，D-下
*/
typedef enum lcd_scan_dir_en {
    L2R_U2D = 0,
    L2R_D2U = 1,
    R2L_U2D = 2,
    R2L_D2U = 3,
    U2D_L2R = 4,
    U2D_R2L = 5,
    D2U_L2R = 6,
    D2U_R2L = 7,
} lcd_scan_dir_t;

/**
 * @brief    LCD显示方向
 * @param    VERTICAL_DISP   竖屏显示
 * @param    HORIZONTAL_DISP 横屏显示
*/
typedef enum lcd_display_dir_en {
    VERTICAL_DISP   = 0,
    HORIZONTAL_DISP = 1,
} lcd_display_dir_t;

#define    BLACK   0x0000    //黑色
#define    BLUE    0x001F    //蓝色
#define    GREEN   0x07E0    //绿色
#define    GBLUE   0X07FF    //天蓝色
#define    CYAN    0x7FFF    //浅蓝色
#define    GRAY    0X8430    //灰色
#define    BROWN   0XBC40    //棕色
#define    RED     0xF800    //红色
#define    BRED    0XF81F    //粉色
#define    BRRED   0XFC07    //棕红色  
#define    YELLOW  0xFFE0    //黄色
#define    WHITE   0xFFFF    //白色

/****************** User Config Start ****************/

/* 使能此驱动是否打印调试日志（需要printf支持） */
#define LCD_LOG_ENABLE          1

/* 屏幕大小（竖屏显示下的值） */
#define LCD_WIDTH               480
#define LCD_HEIGHT              800

/* 背光控制引脚 */
#define LCD_BL_GPIO_Port        GPIOB
#define LCD_BL_Pin              GPIO_PIN_15

/* 通过地址线控制RS引脚 */
#define LCD_CMD_ADDR            0x6c000040
#define LCD_DAT_ADDR            0x6c000080

/* LCD默认显示方向:VERTICAL_DISP-竖屏，HORIZONTAL_DISP-横屏 */
#define LCD_DEFAULT_DISPLAY_DIR VERTICAL_DISP

/* LCD初始化清屏颜色 */
#define LCD_DEFAULT_CLEAR_COLOR WHITE

/****************** User Config End ****************/

#if LCD_LOG_ENABLE
#include <stdio.h>
#define LCD_LOG printf
#else
#define LCD_LOG(format,...)
#endif

extern lcd_params_t lcd_params;

void lcd_init(void);
void lcd_display_on(void);
void lcd_display_off(void);
void lcd_backlight_ctrl(lcd_backlight_state_t state);
void lcd_clear(uint16_t color);

void lcd_draw_point(uint16_t x_pos, uint16_t y_pos, uint16_t color);
void lcd_set_window(uint16_t x_pos_start, uint16_t y_pos_start, uint16_t width, uint16_t height);
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcd_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcd_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);
void lcd_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcd_fill_with_buffer(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color_buf);

#endif /* _LCD_FSMC_H_ */
