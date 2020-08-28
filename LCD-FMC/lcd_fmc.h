/**
 * @file    lcd_fmc.c
 * @brief   使用FMC外设驱动TFT-LCD屏幕（MCU屏）
 * @author  Mculover666
 * @date    2020/08/27
*/

#ifndef _LCD_FMC_H_
#define _LCD_FMC_H_

#include "fmc.h"

typedef struct lcd_fmc_address_st {
    volatile uint16_t lcd_reg;
    volatile uint16_t lcd_ram;
} lcd_fmc_address_t;

/**
 * @brief    保存LCD屏幕参数
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
 * @brief    LCD扫描方向枚举
 * @note     L-左，R-右，U-上，D-下
*/
typedef enum lcd_scan_dir_en {
    L2R_U2D = 0,
    L2R_D2U,
    R2L_U2D,
    R2L_D2U,
    U2D_L2R,
    U2D_R2L,
    D2U_L2R,
    D2U_R2L
} lcd_scan_dir_t;

/**
 * @brief    LCD颜色
*/
typedef enum lcd_color_en {
    BLACK   = 0x0000, //黑色
    BLUE    = 0x001F, //蓝色
    GREEN   = 0x07E0, //绿色
    GBLUE	= 0X07FF, //天蓝色
    CYAN    = 0x7FFF, //浅蓝色
    GRAY  	= 0X8430, //灰色
    BROWN 	= 0XBC40, //棕色
    RED     = 0xF800, //红色
    BRED    = 0XF81F, //粉色
    BRRED 	= 0XFC07, //棕红色  
    YELLOW  = 0xFFE0, //黄色
    WHITE   = 0xFFFF, //白色
} lcd_color_t;

/*
    作用：
    - 操作 LCD->lcd_reg 地址处的数据，则FMC外设发出信号时，A18=0，即LCD_RS=0
    - 操作 LCD->lcd_ram 地址处的数据，则FMC外设发出信号时，A18=1，即LCD_RS=1
*/
#define LCD_BASE    ((uint32_t)(0x60000000 | 0x0007FFFE))
#define LCD         ((lcd_fmc_address_t*)LCD_BASE)

/* LCD MPU保护参数 */
#define LCD_REGION_NUMBER		MPU_REGION_NUMBER0		//LCD使用region0
#define LCD_ADDRESS_START		(0X60000000)			//LCD区的首地址
#define LCD_REGION_SIZE			MPU_REGION_SIZE_256MB   //LCD区大小

/* 使能此驱动是否打印调试日志（需要printf支持） */
#define LCD_LOG_ENABLE          1

#if LCD_LOG_ENABLE
#include <stdio.h>
#define LCD_LOG printf
#else
#define LCD_LOG(format,...)
#endif

/* 屏幕大小（竖屏显示下的值） */
#define LCD_WIDTH   480
#define LCD_HEIGHT  800

extern lcd_params_t lcd_params;

void lcd_init(void);
void lcd_display_on(void);
void lcd_display_off(void);
void lcd_backlight_on(void);
void lcd_backlight_off(void);
void lcd_clear(lcd_color_t color);

#endif /* _LCD_FMC_H_ */
