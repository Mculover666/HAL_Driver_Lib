# 示例
## 基本使用
包含头文件：
```c
#include "lcd_spi_drv.h"
```
初始化屏幕：
```c
lcd_init();
```
清屏：
```c
lcd_clear(WHITE);
```

## 画线示例
```c
lcd_draw_color_line(0,120,240,120,RED);		//画水平线
lcd_draw_color_line(0,0,240,240,BLUE);	  	//画斜线(从左到右，45°)
lcd_draw_color_line(0,240,240,0,GREEN);		//画斜线(从右到左，45°)
lcd_draw_color_line(120,0,120,240,YELLOW);	//画垂直线
lcd_draw_color_line(180,0,60,240,RED);		//画斜线(从左到右，120°)
lcd_draw_color_line(60,0,180,240,RED);		//画斜线(从右到左，60°)
lcd_draw_color_line(0,60,240,180,RED);		//画斜线(从左到右，180°)
lcd_draw_color_line(0,180,240,60,RED);		//画斜线(从左到右，30°)
LCD_Draw_ColorRect(60,60,180,180,PINK);		//画矩形
LCD_Draw_ColorCircle(120,120,85, GBLUE);	//画圆
```

## 字符示例
需要开启字符显示：
```c
#define	USE_ASCII_FONT_LIB			1   // Whether to enable character display
```
测试代码：
```c
//以下9行测试12/16/24/32四种字符和字符串显示
lcd_show_char(6,12,'B',BLACK,YELLOW,16);
lcd_show_char(14,28,'C',BLACK,GREEN,24);
lcd_show_char(0,0,'A',BLACK,BLUE,12);
lcd_show_char(26,52,'D',BLACK,PINK,32);
lcd_show_str(60,240-32-24-24-24,"Powerd BY",BLACK,GREEN,24);
lcd_show_str(36,240-32-24-24,"TencentOS-tiny",BLACK,YELLOW,24);
lcd_show_str(28,240-32-24,"Mculover666",BLACK,BLUE,32);
lcd_show_str(12,240-24,"Copyright (c) 2019",BLACK,PINK,24);=
```
## 六芒星示例
```c
lcd_draw_color_sixpointstar(150,65,40,RED);
```

## 图片示例
需要开启字符显示：
```c
#define	USE_ASCII_FONT_LIB			1   // Whether to enable character display
```
包含图片头文件：
```c
#include "bear.h"
```
图片显示测试：
```c
//以下2行测试图片显示，需要bear.h头文件的支持
lcd_show_image(0,0,240,240,gImage_bear);
lcd_show_str(70,240-24,"Starting...",WHITE,BLUE,24);
```

## 刷屏示例
```c
start_time = HAL_GetTick();
lcd_color_set(GREEN, WHITE);
lcd_clear();
end_time = HAL_GetTick();
printf("clear time is %d ms\r\n", end_time - start_time);
```