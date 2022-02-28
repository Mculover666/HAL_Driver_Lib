/******************************************************
 * @file    lcd_fsmc.c
 * @brief   使用FSMC外设驱动TFT-LCD屏幕（MCU屏）
 * @author  Mculover666
 * @date    2020/08/27
 * @note    
 *          目前支持的驱动器：
 *            - ILI9341(0x9341)
 *            - NT35310(0x5310)
 *            - NT35510(0x5510)
******************************************************/

#include "lcd_fsmc.h"

lcd_params_t lcd_params;

/**
 * @brief    背光控制
 * @param    state 背光状态
 * @retval   none
*/
void lcd_backlight_ctrl(lcd_backlight_state_t state)
{
    if (state == LCD_BACKLIGHT_ON) {
        HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);
    }
    
}

/**
 * @brief    向LCD写入命令
 * @param    cmd 待写入命令
 * @retval   none
*/
static void lcd_write_cmd(__IO uint16_t cmd)
{
    *(uint16_t *)(LCD_CMD_ADDR) = cmd;
}

/**
 * @brief    向LCD写入数据
 * @param    data 待写入数据
 * @retval   none
*/
static void lcd_write_data(__IO uint16_t data)
{
    *(uint16_t *)(LCD_DAT_ADDR) = data;
}

/**
 * @brief    从LCD读取数据
 * @param    none
 * @retval   读取到的数据
*/
static uint16_t lcd_read_data(void)
{
    __IO uint16_t data;
    
    data = *(uint16_t *)(LCD_DAT_ADDR);
    
    return data;
}

/**
 * @brief    写LCD中的寄存器
 * @param    reg  寄存器序号
 * @param    data 要写入寄存器的值
 * @retval   none
*/
static void lcd_write_reg(__IO uint16_t reg, __IO uint16_t data)
{
    lcd_write_cmd(reg);
    lcd_write_data(data);
}

/**
 * @brief    读取LCD控制IC的ID，存入LCD参数结构体
 * @param    none
 * @retval   成功 0
 * @retval   失败 -1
*/
static int lcd_read_id(void)
{
    /* 尝试执行ILI9341控制器ID的读取流程 */
    lcd_write_cmd(0xD3);				   
	lcd_params.lcd_id = lcd_read_data();
	lcd_params.lcd_id = lcd_read_data();
	lcd_params.lcd_id = lcd_read_data();				   
	lcd_params.lcd_id <<= 8;
	lcd_params.lcd_id |= lcd_read_data();
    /* 如果正常读到，则返回成功 */
    if (lcd_params.lcd_id == 0x9341) {
        return 0;
    }
    
    /* 尝试执行NT35310控制器ID的读取流程 */
    lcd_write_cmd(0xD4);				   
    lcd_params.lcd_id = lcd_read_data();
    lcd_params.lcd_id = lcd_read_data();
    lcd_params.lcd_id = lcd_read_data();
    lcd_params.lcd_id <<= 8;	 
    lcd_params.lcd_id |= lcd_read_data();
    /* 如果正常读到，则返回成功 */
    if (lcd_params.lcd_id == 0x5310) {
        return 0;
    }
    
    /* 尝试执行NT35510控制器ID的读取流程 */
    lcd_write_cmd(0xDA00);
    lcd_params.lcd_id = lcd_read_data();
    lcd_write_cmd(0xDB00);
    lcd_params.lcd_id = lcd_read_data();
    lcd_params.lcd_id <<= 8;	 
    lcd_write_cmd(0xDC00);
    lcd_params.lcd_id |= lcd_read_data();
    /* 如果正常读到，则返回成功 */
    if (lcd_params.lcd_id == 0x8000) {
        lcd_params.lcd_id = 0x5510;
        return 0;
    }
    
    /* 如果需要添加更多驱动支持，请在此处添加读取ID代码 */
   
    /* 驱动IC不支持 */
    return -1;
}

/**
 * @brief    LCD开显示
 * @param    none
 * @retval   none
*/
void lcd_display_on(void)
{
    /* 如果需要添加更多驱动支持，请在此处添加开显示代码 */    
    
	if (lcd_params.lcd_id == 0x9341 || lcd_params.lcd_id == 0x5310) {
        lcd_write_cmd(0x29);
    } else if (lcd_params.lcd_id == 0x5510) {
        lcd_write_cmd(0x2900);
    }
}	 

/**
 * @brief    LCD关显示
 * @param    none
 * @retval   none
*/
void lcd_display_off(void)
{
    /* 如果需要添加更多驱动支持，请在此处添加开显示代码 */    
    
 	if (lcd_params.lcd_id == 0x9341 || lcd_params.lcd_id == 0x5310) {
        lcd_write_cmd(0x28);
    } else if (lcd_params.lcd_id == 0x5510) {
        lcd_write_cmd(0x2800);
    }
}

/**
 * @brief    LCD设置自动扫描方向
 * @param    dir 扫描方向
 * @retval   none
*/
static void lcd_set_scan_direction(lcd_scan_dir_t dir)
{
    uint16_t regval = 0;
	uint16_t dirreg = 0;
    uint16_t temp = 0;
    
    /* 横屏时，只有1963驱动器不改变扫描方向，其余都要变 */
    if (lcd_params.lcd_direction == 1) {
        switch (dir) {
            case L2R_U2D:   dir = D2U_L2R;    break;
			case L2R_D2U:   dir = D2U_R2L;    break;
			case R2L_U2D:   dir = U2D_L2R;    break;
			case R2L_D2U:   dir = U2D_R2L;    break;	 
			case U2D_L2R:   dir = L2R_D2U;    break;
			case U2D_R2L:   dir = L2R_U2D;    break;
			case D2U_L2R:   dir = R2L_D2U;    break;
			case D2U_R2L:   dir = R2L_U2D;    break;	 
        }
    }
    
	if (lcd_params.lcd_id == 0x9341 || lcd_params.lcd_id == 0x5310 || lcd_params.lcd_id == 0x5510) {
		switch (dir) {
			case L2R_U2D:
				regval |= (0<<7)|(0<<6)|(0<<5); 
				break;
			case L2R_D2U:
				regval |= (1<<7)|(0<<6)|(0<<5); 
				break;
			case R2L_U2D:
				regval |= (0<<7)|(1<<6)|(0<<5); 
				break;
			case R2L_D2U:
				regval |= (1<<7)|(1<<6)|(0<<5); 
				break;	 
			case U2D_L2R:
				regval |= (0<<7)|(0<<6)|(1<<5); 
				break;
			case U2D_R2L:
				regval |= (0<<7)|(1<<6)|(1<<5); 
				break;
			case D2U_L2R:
				regval |= (1<<7)|(0<<6)|(1<<5); 
				break;
			case D2U_R2L:
				regval |= (1<<7)|(1<<6)|(1<<5); 
				break;	 
		}
        
		if (lcd_params.lcd_id == 0x5510) {
            dirreg = 0x3600;
        } else {
            dirreg = 0x36;
        }
        
 		if ((lcd_params.lcd_id != 0x5310) && (lcd_params.lcd_id != 0X5510)) {
            regval |= 0X08;
        }
        
        lcd_write_cmd(dirreg);
        lcd_write_data(regval);
 		
        if(regval & 0x20) {
            if(lcd_params.lcd_width < lcd_params.lcd_height) {
                temp = lcd_params.lcd_width;
                lcd_params.lcd_width = lcd_params.lcd_height;
                lcd_params.lcd_height = temp;
            }
        } else {
            if (lcd_params.lcd_width > lcd_params.lcd_height) {
                temp = lcd_params.lcd_width;
                lcd_params.lcd_width = lcd_params.lcd_height;
                lcd_params.lcd_height = temp;
            }
        }
	
		if (lcd_params.lcd_id == 0x5510) {
			lcd_write_cmd(lcd_params.set_x_cmd);
            lcd_write_data(0); 
			lcd_write_cmd(lcd_params.set_x_cmd + 1);
            lcd_write_data(0); 
			lcd_write_cmd(lcd_params.set_x_cmd + 2);
            lcd_write_data((lcd_params.lcd_width - 1) >> 8); 
			lcd_write_cmd(lcd_params.set_x_cmd + 3);
            lcd_write_data((lcd_params.lcd_width - 1) & 0xFF); 
			lcd_write_cmd(lcd_params.set_y_cmd);
            lcd_write_data(0); 
			lcd_write_cmd(lcd_params.set_y_cmd + 1);
            lcd_write_data(0); 
			lcd_write_cmd(lcd_params.set_y_cmd + 2);
            lcd_write_data((lcd_params.lcd_height - 1) >> 8); 
			lcd_write_cmd(lcd_params.set_y_cmd + 3);
            lcd_write_data((lcd_params.lcd_height - 1) & 0xFF);
		} else {
			lcd_write_cmd(lcd_params.set_x_cmd); 
			lcd_write_data(0);
            lcd_write_data(0);
			lcd_write_data((lcd_params.lcd_width - 1) >> 8);
            lcd_write_data((lcd_params.lcd_width - 1) & 0xFF);
			lcd_write_cmd(lcd_params.set_y_cmd); 
			lcd_write_data(0);
            lcd_write_data(0);
			lcd_write_data((lcd_params.lcd_height - 1) >> 8);
            lcd_write_data((lcd_params.lcd_height - 1) & 0xFF);  
		}   
  	}
}

/**
 * @brief    LCD设置显示方向
 * @param    dir 显示方向
 * @retval   none
*/
static void lcd_set_display_drection(lcd_display_dir_t dir)
{
    if (dir == VERTICAL_DISP) {
        /* 设置竖屏 */
        lcd_params.lcd_direction = 0;
        lcd_params.lcd_width     = LCD_WIDTH;
        lcd_params.lcd_height    = LCD_HEIGHT;
        
        if(lcd_params.lcd_id==0X9341||lcd_params.lcd_id==0X6804||lcd_params.lcd_id==0X5310)
		{
			lcd_params.wram_cmd=0X2C;
	 		lcd_params.set_x_cmd=0X2A;
			lcd_params.set_y_cmd=0X2B;  	 
			if(lcd_params.lcd_id==0X6804||lcd_params.lcd_id==0X5310)
			{
				lcd_params.lcd_width=LCD_WIDTH;
				lcd_params.lcd_height=LCD_HEIGHT;
			}
		}else if(lcd_params.lcd_id==0x5510)
		{
			lcd_params.wram_cmd=0X2C00;
	 		lcd_params.set_x_cmd=0X2A00;
			lcd_params.set_y_cmd=0X2B00; 
			lcd_params.lcd_width=480;
			lcd_params.lcd_height=800;
        }
    } else {
        /* 设置横屏 */
        lcd_params.lcd_direction = 1;
        lcd_params.lcd_width     = LCD_HEIGHT;
        lcd_params.lcd_height    = LCD_WIDTH;
        
         if(lcd_params.lcd_id==0X9341||lcd_params.lcd_id==0X6804||lcd_params.lcd_id==0X5310)
		{
			lcd_params.wram_cmd=0X2C;
	 		lcd_params.set_x_cmd=0X2A;
			lcd_params.set_y_cmd=0X2B;  	 
			if(lcd_params.lcd_id==0X6804||lcd_params.lcd_id==0X5310)
			{
				lcd_params.lcd_width=LCD_WIDTH;
				lcd_params.lcd_height=LCD_HEIGHT;
			}
		}else if(lcd_params.lcd_id==0x5510)
		{
			lcd_params.wram_cmd=0X2C00;
	 		lcd_params.set_x_cmd=0X2A00;
			lcd_params.set_y_cmd=0X2B00; 
			lcd_params.lcd_width=800;
			lcd_params.lcd_height=480;
        }
    }
     
    /* 设置扫描方向 */
    lcd_set_scan_direction(L2R_U2D);
}

/**
 * @brief    LCD设置光标位置
 * @param    x_pos x方向坐标
 * @param    y_pos yu方向坐标
 * @retval   none
*/
static void lcd_set_cursor(uint16_t x_pos, uint16_t y_pos)
{
    /* 如果需要添加更多驱动支持，请在此处添加设置光标位置代码 */ 
    
 	if (lcd_params.lcd_id == 0x9341 || lcd_params.lcd_id == 0x5310) {	    
		lcd_write_cmd(lcd_params.set_x_cmd); 
		lcd_write_data(x_pos >> 8);
        lcd_write_data(x_pos & 0xFF); 			 
		lcd_write_cmd(lcd_params.set_y_cmd); 
		lcd_write_data(y_pos >> 8);
        lcd_write_data(y_pos & 0xFF); 		
	} else if (lcd_params.lcd_id == 0x5510) {
		lcd_write_cmd(lcd_params.set_x_cmd);
        lcd_write_data(x_pos >> 8); 		
		lcd_write_cmd(lcd_params.set_x_cmd + 1);
        lcd_write_data(x_pos & 0xFF);			 
		lcd_write_cmd(lcd_params.set_y_cmd);
        lcd_write_data(y_pos >> 8);  		
		lcd_write_cmd(lcd_params.set_y_cmd + 1);
        lcd_write_data(y_pos & 0xFF);			
	}
}

/**
 * @brief    LCD开始写GRAM
 * @param    none
 * @retval   none
*/
static void lcd_write_ram_start(void)
{
    lcd_write_cmd(lcd_params.wram_cmd);
}

/**
 * @brief    LCD写GRAM
 * @param    rgb_color 颜色值
 * @retval   none
*/
static void lcd_write_ram(uint16_t rgb_color)
{
    lcd_write_data(rgb_color);
}

/**
 * @brief    LCD清屏函数
 * @param    color 清屏颜色
 * @retval   none
*/
void lcd_clear(uint16_t color)
{
	uint32_t index = 0;      
	uint32_t totalpoint = 0;
    
    /* 计算得到总点数 */
	totalpoint = lcd_params.lcd_width * lcd_params.lcd_height;
    
    /* 设置光标位置 */
	lcd_set_cursor(0x00,0x0000);
    
    /* 开始写入GRAM */
	lcd_write_ram_start();
    
    /* 写入数据到GRAM */
	for (index = 0; index < totalpoint; index++) {
		lcd_write_ram(color);
	}
}

/**
 * @brief    LCD初始化
 * @param    none
 * @retval   none
*/
void lcd_init(void)
{
    /* 初始化FMC接口 */
    //MX_FMC_Init();
	
 	HAL_Delay(50); 
	
    /* 读取LCD控制器IC */
    if (lcd_read_id() == -1) {
        LCD_LOG("Read ID Fail, Not Support LCD IC:%#x!\r\n", lcd_params.lcd_id);
        return;
    } else {
        LCD_LOG("LCD IC ID is:%#x\r\n", lcd_params.lcd_id);
    }
    
    /*
      根据不同的驱动ID进行LCD初始化
      目前支持：
        - 0x9341
        - 0x5310
        - 0x5510
    */
    if (lcd_params.lcd_id == 0x9341) {
        lcd_write_cmd(0xCF);  
        lcd_write_data(0x00); 
        lcd_write_data(0xC1); 
        lcd_write_data(0X30); 
        lcd_write_cmd(0xED);  
        lcd_write_data(0x64); 
        lcd_write_data(0x03); 
        lcd_write_data(0X12); 
        lcd_write_data(0X81); 
        lcd_write_cmd(0xE8);  
        lcd_write_data(0x85); 
        lcd_write_data(0x10); 
        lcd_write_data(0x7A); 
        lcd_write_cmd(0xCB);  
        lcd_write_data(0x39); 
        lcd_write_data(0x2C); 
        lcd_write_data(0x00); 
        lcd_write_data(0x34); 
        lcd_write_data(0x02); 
        lcd_write_cmd(0xF7);  
        lcd_write_data(0x20); 
        lcd_write_cmd(0xEA);  
        lcd_write_data(0x00); 
        lcd_write_data(0x00); 
        lcd_write_cmd(0xC0);
        lcd_write_data(0x1B);
        lcd_write_cmd(0xC1);
        lcd_write_data(0x01);
        lcd_write_cmd(0xC5);
        lcd_write_data(0x30);
        lcd_write_data(0x30);
        lcd_write_cmd(0xC7);
        lcd_write_data(0XB7); 
        lcd_write_cmd(0x36);
        lcd_write_data(0x48); 
        lcd_write_cmd(0x3A);   
        lcd_write_data(0x55); 
        lcd_write_cmd(0xB1);   
        lcd_write_data(0x00);   
        lcd_write_data(0x1A); 
        lcd_write_cmd(0xB6);
        lcd_write_data(0x0A); 
        lcd_write_data(0xA2); 
        lcd_write_cmd(0xF2);
        lcd_write_data(0x00); 
        lcd_write_cmd(0x26);
        lcd_write_data(0x01); 
        lcd_write_cmd(0xE0);
        lcd_write_data(0x0F); 
        lcd_write_data(0x2A); 
        lcd_write_data(0x28); 
        lcd_write_data(0x08); 
        lcd_write_data(0x0E); 
        lcd_write_data(0x08); 
        lcd_write_data(0x54); 
        lcd_write_data(0XA9); 
        lcd_write_data(0x43); 
        lcd_write_data(0x0A); 
        lcd_write_data(0x0F); 
        lcd_write_data(0x00); 
        lcd_write_data(0x00); 
        lcd_write_data(0x00); 
        lcd_write_data(0x00); 		 
        lcd_write_cmd(0XE1);
        lcd_write_data(0x00); 
        lcd_write_data(0x15); 
        lcd_write_data(0x17); 
        lcd_write_data(0x07); 
        lcd_write_data(0x11); 
        lcd_write_data(0x06); 
        lcd_write_data(0x2B); 
        lcd_write_data(0x56); 
        lcd_write_data(0x3C); 
        lcd_write_data(0x05); 
        lcd_write_data(0x10); 
        lcd_write_data(0x0F); 
        lcd_write_data(0x3F); 
        lcd_write_data(0x3F); 
        lcd_write_data(0x0F); 
        lcd_write_cmd(0x2B); 
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x01);
        lcd_write_data(0x3f);
        lcd_write_cmd(0x2A); 
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0xef);	 
        lcd_write_cmd(0x11);
        HAL_Delay(120);
        lcd_write_cmd(0x29);
    } else if (lcd_params.lcd_id == 0x5310) {
        lcd_write_cmd(0xED);
        lcd_write_data(0x01);
        lcd_write_data(0xFE);

        lcd_write_cmd(0xEE);
        lcd_write_data(0xDE);
        lcd_write_data(0x21);

        lcd_write_cmd(0xF1);
        lcd_write_data(0x01);
        lcd_write_cmd(0xDF);
        lcd_write_data(0x10);

        lcd_write_cmd(0xC4);
        lcd_write_data(0x8F);

        lcd_write_cmd(0xC6);
        lcd_write_data(0x00);
        lcd_write_data(0xE2);
        lcd_write_data(0xE2);
        lcd_write_data(0xE2);
        lcd_write_cmd(0xBF);
        lcd_write_data(0xAA);

        lcd_write_cmd(0xB0);
        lcd_write_data(0x0D);
        lcd_write_data(0x00);
        lcd_write_data(0x0D);
        lcd_write_data(0x00);
        lcd_write_data(0x11);
        lcd_write_data(0x00);
        lcd_write_data(0x19);
        lcd_write_data(0x00);
        lcd_write_data(0x21);
        lcd_write_data(0x00);
        lcd_write_data(0x2D);
        lcd_write_data(0x00);
        lcd_write_data(0x3D);
        lcd_write_data(0x00);
        lcd_write_data(0x5D);
        lcd_write_data(0x00);
        lcd_write_data(0x5D);
        lcd_write_data(0x00);

        lcd_write_cmd(0xB1);
        lcd_write_data(0x80);
        lcd_write_data(0x00);
        lcd_write_data(0x8B);
        lcd_write_data(0x00);
        lcd_write_data(0x96);
        lcd_write_data(0x00);

        lcd_write_cmd(0xB2);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x02);
        lcd_write_data(0x00);
        lcd_write_data(0x03);
        lcd_write_data(0x00);

        lcd_write_cmd(0xB3);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);

        lcd_write_cmd(0xB4);
        lcd_write_data(0x8B);
        lcd_write_data(0x00);
        lcd_write_data(0x96);
        lcd_write_data(0x00);
        lcd_write_data(0xA1);
        lcd_write_data(0x00);

        lcd_write_cmd(0xB5);
        lcd_write_data(0x02);
        lcd_write_data(0x00);
        lcd_write_data(0x03);
        lcd_write_data(0x00);
        lcd_write_data(0x04);
        lcd_write_data(0x00);

        lcd_write_cmd(0xB6);
        lcd_write_data(0x00);
        lcd_write_data(0x00);

        lcd_write_cmd(0xB7);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x3F);
        lcd_write_data(0x00);
        lcd_write_data(0x5E);
        lcd_write_data(0x00);
        lcd_write_data(0x64);
        lcd_write_data(0x00);
        lcd_write_data(0x8C);
        lcd_write_data(0x00);
        lcd_write_data(0xAC);
        lcd_write_data(0x00);
        lcd_write_data(0xDC);
        lcd_write_data(0x00);
        lcd_write_data(0x70);
        lcd_write_data(0x00);
        lcd_write_data(0x90);
        lcd_write_data(0x00);
        lcd_write_data(0xEB);
        lcd_write_data(0x00);
        lcd_write_data(0xDC);
        lcd_write_data(0x00);

        lcd_write_cmd(0xB8);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);

        lcd_write_cmd(0xBA);
        lcd_write_data(0x24);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);

        lcd_write_cmd(0xC1);
        lcd_write_data(0x20);
        lcd_write_data(0x00);
        lcd_write_data(0x54);
        lcd_write_data(0x00);
        lcd_write_data(0xFF);
        lcd_write_data(0x00);

        lcd_write_cmd(0xC2);
        lcd_write_data(0x0A);
        lcd_write_data(0x00);
        lcd_write_data(0x04);
        lcd_write_data(0x00);

        lcd_write_cmd(0xC3);
        lcd_write_data(0x3C);
        lcd_write_data(0x00);
        lcd_write_data(0x3A);
        lcd_write_data(0x00);
        lcd_write_data(0x39);
        lcd_write_data(0x00);
        lcd_write_data(0x37);
        lcd_write_data(0x00);
        lcd_write_data(0x3C);
        lcd_write_data(0x00);
        lcd_write_data(0x36);
        lcd_write_data(0x00);
        lcd_write_data(0x32);
        lcd_write_data(0x00);
        lcd_write_data(0x2F);
        lcd_write_data(0x00);
        lcd_write_data(0x2C);
        lcd_write_data(0x00);
        lcd_write_data(0x29);
        lcd_write_data(0x00);
        lcd_write_data(0x26);
        lcd_write_data(0x00);
        lcd_write_data(0x24);
        lcd_write_data(0x00);
        lcd_write_data(0x24);
        lcd_write_data(0x00);
        lcd_write_data(0x23);
        lcd_write_data(0x00);
        lcd_write_data(0x3C);
        lcd_write_data(0x00);
        lcd_write_data(0x36);
        lcd_write_data(0x00);
        lcd_write_data(0x32);
        lcd_write_data(0x00);
        lcd_write_data(0x2F);
        lcd_write_data(0x00);
        lcd_write_data(0x2C);
        lcd_write_data(0x00);
        lcd_write_data(0x29);
        lcd_write_data(0x00);
        lcd_write_data(0x26);
        lcd_write_data(0x00);
        lcd_write_data(0x24);
        lcd_write_data(0x00);
        lcd_write_data(0x24);
        lcd_write_data(0x00);
        lcd_write_data(0x23);
        lcd_write_data(0x00);

        lcd_write_cmd(0xC4);
        lcd_write_data(0x62);
        lcd_write_data(0x00);
        lcd_write_data(0x05);
        lcd_write_data(0x00);
        lcd_write_data(0x84);
        lcd_write_data(0x00);
        lcd_write_data(0xF0);
        lcd_write_data(0x00);
        lcd_write_data(0x18);
        lcd_write_data(0x00);
        lcd_write_data(0xA4);
        lcd_write_data(0x00);
        lcd_write_data(0x18);
        lcd_write_data(0x00);
        lcd_write_data(0x50);
        lcd_write_data(0x00);
        lcd_write_data(0x0C);
        lcd_write_data(0x00);
        lcd_write_data(0x17);
        lcd_write_data(0x00);
        lcd_write_data(0x95);
        lcd_write_data(0x00);
        lcd_write_data(0xF3);
        lcd_write_data(0x00);
        lcd_write_data(0xE6);
        lcd_write_data(0x00);

        lcd_write_cmd(0xC5);
        lcd_write_data(0x32);
        lcd_write_data(0x00);
        lcd_write_data(0x44);
        lcd_write_data(0x00);
        lcd_write_data(0x65);
        lcd_write_data(0x00);
        lcd_write_data(0x76);
        lcd_write_data(0x00);
        lcd_write_data(0x88);
        lcd_write_data(0x00);

        lcd_write_cmd(0xC6);
        lcd_write_data(0x20);
        lcd_write_data(0x00);
        lcd_write_data(0x17);
        lcd_write_data(0x00);
        lcd_write_data(0x01);
        lcd_write_data(0x00);

        lcd_write_cmd(0xC7);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);

        lcd_write_cmd(0xC8);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);

        lcd_write_cmd(0xC9);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);

        lcd_write_cmd(0xE0);
        lcd_write_data(0x16);
        lcd_write_data(0x00);
        lcd_write_data(0x1C);
        lcd_write_data(0x00);
        lcd_write_data(0x21);
        lcd_write_data(0x00);
        lcd_write_data(0x36);
        lcd_write_data(0x00);
        lcd_write_data(0x46);
        lcd_write_data(0x00);
        lcd_write_data(0x52);
        lcd_write_data(0x00);
        lcd_write_data(0x64);
        lcd_write_data(0x00);
        lcd_write_data(0x7A);
        lcd_write_data(0x00);
        lcd_write_data(0x8B);
        lcd_write_data(0x00);
        lcd_write_data(0x99);
        lcd_write_data(0x00);
        lcd_write_data(0xA8);
        lcd_write_data(0x00);
        lcd_write_data(0xB9);
        lcd_write_data(0x00);
        lcd_write_data(0xC4);
        lcd_write_data(0x00);
        lcd_write_data(0xCA);
        lcd_write_data(0x00);
        lcd_write_data(0xD2);
        lcd_write_data(0x00);
        lcd_write_data(0xD9);
        lcd_write_data(0x00);
        lcd_write_data(0xE0);
        lcd_write_data(0x00);
        lcd_write_data(0xF3);
        lcd_write_data(0x00);

        lcd_write_cmd(0xE1);
        lcd_write_data(0x16);
        lcd_write_data(0x00);
        lcd_write_data(0x1C);
        lcd_write_data(0x00);
        lcd_write_data(0x22);
        lcd_write_data(0x00);
        lcd_write_data(0x36);
        lcd_write_data(0x00);
        lcd_write_data(0x45);
        lcd_write_data(0x00);
        lcd_write_data(0x52);
        lcd_write_data(0x00);
        lcd_write_data(0x64);
        lcd_write_data(0x00);
        lcd_write_data(0x7A);
        lcd_write_data(0x00);
        lcd_write_data(0x8B);
        lcd_write_data(0x00);
        lcd_write_data(0x99);
        lcd_write_data(0x00);
        lcd_write_data(0xA8);
        lcd_write_data(0x00);
        lcd_write_data(0xB9);
        lcd_write_data(0x00);
        lcd_write_data(0xC4);
        lcd_write_data(0x00);
        lcd_write_data(0xCA);
        lcd_write_data(0x00);
        lcd_write_data(0xD2);
        lcd_write_data(0x00);
        lcd_write_data(0xD8);
        lcd_write_data(0x00);
        lcd_write_data(0xE0);
        lcd_write_data(0x00);
        lcd_write_data(0xF3);
        lcd_write_data(0x00);

        lcd_write_cmd(0xE2);
        lcd_write_data(0x05);
        lcd_write_data(0x00);
        lcd_write_data(0x0B);
        lcd_write_data(0x00);
        lcd_write_data(0x1B);
        lcd_write_data(0x00);
        lcd_write_data(0x34);
        lcd_write_data(0x00);
        lcd_write_data(0x44);
        lcd_write_data(0x00);
        lcd_write_data(0x4F);
        lcd_write_data(0x00);
        lcd_write_data(0x61);
        lcd_write_data(0x00);
        lcd_write_data(0x79);
        lcd_write_data(0x00);
        lcd_write_data(0x88);
        lcd_write_data(0x00);
        lcd_write_data(0x97);
        lcd_write_data(0x00);
        lcd_write_data(0xA6);
        lcd_write_data(0x00);
        lcd_write_data(0xB7);
        lcd_write_data(0x00);
        lcd_write_data(0xC2);
        lcd_write_data(0x00);
        lcd_write_data(0xC7);
        lcd_write_data(0x00);
        lcd_write_data(0xD1);
        lcd_write_data(0x00);
        lcd_write_data(0xD6);
        lcd_write_data(0x00);
        lcd_write_data(0xDD);
        lcd_write_data(0x00);
        lcd_write_data(0xF3);
        lcd_write_data(0x00);
        lcd_write_cmd(0xE3);
        lcd_write_data(0x05);
        lcd_write_data(0x00);
        lcd_write_data(0xA);
        lcd_write_data(0x00);
        lcd_write_data(0x1C);
        lcd_write_data(0x00);
        lcd_write_data(0x33);
        lcd_write_data(0x00);
        lcd_write_data(0x44);
        lcd_write_data(0x00);
        lcd_write_data(0x50);
        lcd_write_data(0x00);
        lcd_write_data(0x62);
        lcd_write_data(0x00);
        lcd_write_data(0x78);
        lcd_write_data(0x00);
        lcd_write_data(0x88);
        lcd_write_data(0x00);
        lcd_write_data(0x97);
        lcd_write_data(0x00);
        lcd_write_data(0xA6);
        lcd_write_data(0x00);
        lcd_write_data(0xB7);
        lcd_write_data(0x00);
        lcd_write_data(0xC2);
        lcd_write_data(0x00);
        lcd_write_data(0xC7);
        lcd_write_data(0x00);
        lcd_write_data(0xD1);
        lcd_write_data(0x00);
        lcd_write_data(0xD5);
        lcd_write_data(0x00);
        lcd_write_data(0xDD);
        lcd_write_data(0x00);
        lcd_write_data(0xF3);
        lcd_write_data(0x00);

        lcd_write_cmd(0xE4);
        lcd_write_data(0x01);
        lcd_write_data(0x00);
        lcd_write_data(0x01);
        lcd_write_data(0x00);
        lcd_write_data(0x02);
        lcd_write_data(0x00);
        lcd_write_data(0x2A);
        lcd_write_data(0x00);
        lcd_write_data(0x3C);
        lcd_write_data(0x00);
        lcd_write_data(0x4B);
        lcd_write_data(0x00);
        lcd_write_data(0x5D);
        lcd_write_data(0x00);
        lcd_write_data(0x74);
        lcd_write_data(0x00);
        lcd_write_data(0x84);
        lcd_write_data(0x00);
        lcd_write_data(0x93);
        lcd_write_data(0x00);
        lcd_write_data(0xA2);
        lcd_write_data(0x00);
        lcd_write_data(0xB3);
        lcd_write_data(0x00);
        lcd_write_data(0xBE);
        lcd_write_data(0x00);
        lcd_write_data(0xC4);
        lcd_write_data(0x00);
        lcd_write_data(0xCD);
        lcd_write_data(0x00);
        lcd_write_data(0xD3);
        lcd_write_data(0x00);
        lcd_write_data(0xDD);
        lcd_write_data(0x00);
        lcd_write_data(0xF3);
        lcd_write_data(0x00);
        lcd_write_cmd(0xE5);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x02);
        lcd_write_data(0x00);
        lcd_write_data(0x29);
        lcd_write_data(0x00);
        lcd_write_data(0x3C);
        lcd_write_data(0x00);
        lcd_write_data(0x4B);
        lcd_write_data(0x00);
        lcd_write_data(0x5D);
        lcd_write_data(0x00);
        lcd_write_data(0x74);
        lcd_write_data(0x00);
        lcd_write_data(0x84);
        lcd_write_data(0x00);
        lcd_write_data(0x93);
        lcd_write_data(0x00);
        lcd_write_data(0xA2);
        lcd_write_data(0x00);
        lcd_write_data(0xB3);
        lcd_write_data(0x00);
        lcd_write_data(0xBE);
        lcd_write_data(0x00);
        lcd_write_data(0xC4);
        lcd_write_data(0x00);
        lcd_write_data(0xCD);
        lcd_write_data(0x00);
        lcd_write_data(0xD3);
        lcd_write_data(0x00);
        lcd_write_data(0xDC);
        lcd_write_data(0x00);
        lcd_write_data(0xF3);
        lcd_write_data(0x00);

        lcd_write_cmd(0xE6);
        lcd_write_data(0x11);
        lcd_write_data(0x00);
        lcd_write_data(0x34);
        lcd_write_data(0x00);
        lcd_write_data(0x56);
        lcd_write_data(0x00);
        lcd_write_data(0x76);
        lcd_write_data(0x00);
        lcd_write_data(0x77);
        lcd_write_data(0x00);
        lcd_write_data(0x66);
        lcd_write_data(0x00);
        lcd_write_data(0x88);
        lcd_write_data(0x00);
        lcd_write_data(0x99);
        lcd_write_data(0x00);
        lcd_write_data(0xBB);
        lcd_write_data(0x00);
        lcd_write_data(0x99);
        lcd_write_data(0x00);
        lcd_write_data(0x66);
        lcd_write_data(0x00);
        lcd_write_data(0x55);
        lcd_write_data(0x00);
        lcd_write_data(0x55);
        lcd_write_data(0x00);
        lcd_write_data(0x45);
        lcd_write_data(0x00);
        lcd_write_data(0x43);
        lcd_write_data(0x00);
        lcd_write_data(0x44);
        lcd_write_data(0x00);

        lcd_write_cmd(0xE7);
        lcd_write_data(0x32);
        lcd_write_data(0x00);
        lcd_write_data(0x55);
        lcd_write_data(0x00);
        lcd_write_data(0x76);
        lcd_write_data(0x00);
        lcd_write_data(0x66);
        lcd_write_data(0x00);
        lcd_write_data(0x67);
        lcd_write_data(0x00);
        lcd_write_data(0x67);
        lcd_write_data(0x00);
        lcd_write_data(0x87);
        lcd_write_data(0x00);
        lcd_write_data(0x99);
        lcd_write_data(0x00);
        lcd_write_data(0xBB);
        lcd_write_data(0x00);
        lcd_write_data(0x99);
        lcd_write_data(0x00);
        lcd_write_data(0x77);
        lcd_write_data(0x00);
        lcd_write_data(0x44);
        lcd_write_data(0x00);
        lcd_write_data(0x56);
        lcd_write_data(0x00);
        lcd_write_data(0x23); 
        lcd_write_data(0x00);
        lcd_write_data(0x33);
        lcd_write_data(0x00);
        lcd_write_data(0x45);
        lcd_write_data(0x00);

        lcd_write_cmd(0xE8);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x99);
        lcd_write_data(0x00);
        lcd_write_data(0x87);
        lcd_write_data(0x00);
        lcd_write_data(0x88);
        lcd_write_data(0x00);
        lcd_write_data(0x77);
        lcd_write_data(0x00);
        lcd_write_data(0x66);
        lcd_write_data(0x00);
        lcd_write_data(0x88);
        lcd_write_data(0x00);
        lcd_write_data(0xAA);
        lcd_write_data(0x00);
        lcd_write_data(0xBB);
        lcd_write_data(0x00);
        lcd_write_data(0x99);
        lcd_write_data(0x00);
        lcd_write_data(0x66);
        lcd_write_data(0x00);
        lcd_write_data(0x55);
        lcd_write_data(0x00);
        lcd_write_data(0x55);
        lcd_write_data(0x00);
        lcd_write_data(0x44);
        lcd_write_data(0x00);
        lcd_write_data(0x44);
        lcd_write_data(0x00);
        lcd_write_data(0x55);
        lcd_write_data(0x00);

        lcd_write_cmd(0xE9);
        lcd_write_data(0xAA);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);

        lcd_write_cmd(0x00);
        lcd_write_data(0xAA);

        lcd_write_cmd(0xCF);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);

        lcd_write_cmd(0xF0);
        lcd_write_data(0x00);
        lcd_write_data(0x50);
        lcd_write_data(0x00);
        lcd_write_data(0x00);
        lcd_write_data(0x00);

        lcd_write_cmd(0xF3);
        lcd_write_data(0x00);

        lcd_write_cmd(0xF9);
        lcd_write_data(0x06);
        lcd_write_data(0x10);
        lcd_write_data(0x29);
        lcd_write_data(0x00);

        lcd_write_cmd(0x3A);
        lcd_write_data(0x55);

        lcd_write_cmd(0x11);
        HAL_Delay(100);
        lcd_write_cmd(0x29);
        lcd_write_cmd(0x35);
        lcd_write_data(0x00);

        lcd_write_cmd(0x51);
        lcd_write_data(0xFF);
        lcd_write_cmd(0x53);
        lcd_write_data(0x2C);
        lcd_write_cmd(0x55);
        lcd_write_data(0x82);
        lcd_write_cmd(0x2c);
    } else if (lcd_params.lcd_id == 0x5510) {
        lcd_write_reg(0xF000,0x55);
        lcd_write_reg(0xF001,0xAA);
        lcd_write_reg(0xF002,0x52);
        lcd_write_reg(0xF003,0x08);
        lcd_write_reg(0xF004,0x01);
        lcd_write_reg(0xB000,0x0D);
        lcd_write_reg(0xB001,0x0D);
        lcd_write_reg(0xB002,0x0D);
        lcd_write_reg(0xB600,0x34);
        lcd_write_reg(0xB601,0x34);
        lcd_write_reg(0xB602,0x34);
        lcd_write_reg(0xB100,0x0D);
        lcd_write_reg(0xB101,0x0D);
        lcd_write_reg(0xB102,0x0D);
        lcd_write_reg(0xB700,0x34);
        lcd_write_reg(0xB701,0x34);
        lcd_write_reg(0xB702,0x34);
        lcd_write_reg(0xB200,0x00);
        lcd_write_reg(0xB201,0x00);
        lcd_write_reg(0xB202,0x00);
        lcd_write_reg(0xB800,0x24);
        lcd_write_reg(0xB801,0x24);
        lcd_write_reg(0xB802,0x24);
        lcd_write_reg(0xBF00,0x01);
        lcd_write_reg(0xB300,0x0F);
        lcd_write_reg(0xB301,0x0F);
        lcd_write_reg(0xB302,0x0F);
        lcd_write_reg(0xB900,0x34);
        lcd_write_reg(0xB901,0x34);
        lcd_write_reg(0xB902,0x34);
        lcd_write_reg(0xB500,0x08);
        lcd_write_reg(0xB501,0x08);
        lcd_write_reg(0xB502,0x08);
        lcd_write_reg(0xC200,0x03);
        lcd_write_reg(0xBA00,0x24);
        lcd_write_reg(0xBA01,0x24);
        lcd_write_reg(0xBA02,0x24);
        lcd_write_reg(0xBC00,0x00);
        lcd_write_reg(0xBC01,0x78);
        lcd_write_reg(0xBC02,0x00);
        lcd_write_reg(0xBD00,0x00);
        lcd_write_reg(0xBD01,0x78);
        lcd_write_reg(0xBD02,0x00);
        lcd_write_reg(0xBE00,0x00);
        lcd_write_reg(0xBE01,0x64);
        lcd_write_reg(0xD100,0x00);
        lcd_write_reg(0xD101,0x33);
        lcd_write_reg(0xD102,0x00);
        lcd_write_reg(0xD103,0x34);
        lcd_write_reg(0xD104,0x00);
        lcd_write_reg(0xD105,0x3A);
        lcd_write_reg(0xD106,0x00);
        lcd_write_reg(0xD107,0x4A);
        lcd_write_reg(0xD108,0x00);
        lcd_write_reg(0xD109,0x5C);
        lcd_write_reg(0xD10A,0x00);
        lcd_write_reg(0xD10B,0x81);
        lcd_write_reg(0xD10C,0x00);
        lcd_write_reg(0xD10D,0xA6);
        lcd_write_reg(0xD10E,0x00);
        lcd_write_reg(0xD10F,0xE5);
        lcd_write_reg(0xD110,0x01);
        lcd_write_reg(0xD111,0x13);
        lcd_write_reg(0xD112,0x01);
        lcd_write_reg(0xD113,0x54);
        lcd_write_reg(0xD114,0x01);
        lcd_write_reg(0xD115,0x82);
        lcd_write_reg(0xD116,0x01);
        lcd_write_reg(0xD117,0xCA);
        lcd_write_reg(0xD118,0x02);
        lcd_write_reg(0xD119,0x00);
        lcd_write_reg(0xD11A,0x02);
        lcd_write_reg(0xD11B,0x01);
        lcd_write_reg(0xD11C,0x02);
        lcd_write_reg(0xD11D,0x34);
        lcd_write_reg(0xD11E,0x02);
        lcd_write_reg(0xD11F,0x67);
        lcd_write_reg(0xD120,0x02);
        lcd_write_reg(0xD121,0x84);
        lcd_write_reg(0xD122,0x02);
        lcd_write_reg(0xD123,0xA4);
        lcd_write_reg(0xD124,0x02);
        lcd_write_reg(0xD125,0xB7);
        lcd_write_reg(0xD126,0x02);
        lcd_write_reg(0xD127,0xCF);
        lcd_write_reg(0xD128,0x02);
        lcd_write_reg(0xD129,0xDE);
        lcd_write_reg(0xD12A,0x02);
        lcd_write_reg(0xD12B,0xF2);
        lcd_write_reg(0xD12C,0x02);
        lcd_write_reg(0xD12D,0xFE);
        lcd_write_reg(0xD12E,0x03);
        lcd_write_reg(0xD12F,0x10);
        lcd_write_reg(0xD130,0x03);
        lcd_write_reg(0xD131,0x33);
        lcd_write_reg(0xD132,0x03);
        lcd_write_reg(0xD133,0x6D);
        lcd_write_reg(0xD200,0x00);
        lcd_write_reg(0xD201,0x33);
        lcd_write_reg(0xD202,0x00);
        lcd_write_reg(0xD203,0x34);
        lcd_write_reg(0xD204,0x00);
        lcd_write_reg(0xD205,0x3A);
        lcd_write_reg(0xD206,0x00);
        lcd_write_reg(0xD207,0x4A);
        lcd_write_reg(0xD208,0x00);
        lcd_write_reg(0xD209,0x5C);
        lcd_write_reg(0xD20A,0x00);

        lcd_write_reg(0xD20B,0x81);
        lcd_write_reg(0xD20C,0x00);
        lcd_write_reg(0xD20D,0xA6);
        lcd_write_reg(0xD20E,0x00);
        lcd_write_reg(0xD20F,0xE5);
        lcd_write_reg(0xD210,0x01);
        lcd_write_reg(0xD211,0x13);
        lcd_write_reg(0xD212,0x01);
        lcd_write_reg(0xD213,0x54);
        lcd_write_reg(0xD214,0x01);
        lcd_write_reg(0xD215,0x82);
        lcd_write_reg(0xD216,0x01);
        lcd_write_reg(0xD217,0xCA);
        lcd_write_reg(0xD218,0x02);
        lcd_write_reg(0xD219,0x00);
        lcd_write_reg(0xD21A,0x02);
        lcd_write_reg(0xD21B,0x01);
        lcd_write_reg(0xD21C,0x02);
        lcd_write_reg(0xD21D,0x34);
        lcd_write_reg(0xD21E,0x02);
        lcd_write_reg(0xD21F,0x67);
        lcd_write_reg(0xD220,0x02);
        lcd_write_reg(0xD221,0x84);
        lcd_write_reg(0xD222,0x02);
        lcd_write_reg(0xD223,0xA4);
        lcd_write_reg(0xD224,0x02);
        lcd_write_reg(0xD225,0xB7);
        lcd_write_reg(0xD226,0x02);
        lcd_write_reg(0xD227,0xCF);
        lcd_write_reg(0xD228,0x02);
        lcd_write_reg(0xD229,0xDE);
        lcd_write_reg(0xD22A,0x02);
        lcd_write_reg(0xD22B,0xF2);
        lcd_write_reg(0xD22C,0x02);
        lcd_write_reg(0xD22D,0xFE);
        lcd_write_reg(0xD22E,0x03);
        lcd_write_reg(0xD22F,0x10);
        lcd_write_reg(0xD230,0x03);
        lcd_write_reg(0xD231,0x33);
        lcd_write_reg(0xD232,0x03);
        lcd_write_reg(0xD233,0x6D);
        lcd_write_reg(0xD300,0x00);
        lcd_write_reg(0xD301,0x33);
        lcd_write_reg(0xD302,0x00);
        lcd_write_reg(0xD303,0x34);
        lcd_write_reg(0xD304,0x00);
        lcd_write_reg(0xD305,0x3A);
        lcd_write_reg(0xD306,0x00);
        lcd_write_reg(0xD307,0x4A);
        lcd_write_reg(0xD308,0x00);
        lcd_write_reg(0xD309,0x5C);
        lcd_write_reg(0xD30A,0x00);

        lcd_write_reg(0xD30B,0x81);
        lcd_write_reg(0xD30C,0x00);
        lcd_write_reg(0xD30D,0xA6);
        lcd_write_reg(0xD30E,0x00);
        lcd_write_reg(0xD30F,0xE5);
        lcd_write_reg(0xD310,0x01);
        lcd_write_reg(0xD311,0x13);
        lcd_write_reg(0xD312,0x01);
        lcd_write_reg(0xD313,0x54);
        lcd_write_reg(0xD314,0x01);
        lcd_write_reg(0xD315,0x82);
        lcd_write_reg(0xD316,0x01);
        lcd_write_reg(0xD317,0xCA);
        lcd_write_reg(0xD318,0x02);
        lcd_write_reg(0xD319,0x00);
        lcd_write_reg(0xD31A,0x02);
        lcd_write_reg(0xD31B,0x01);
        lcd_write_reg(0xD31C,0x02);
        lcd_write_reg(0xD31D,0x34);
        lcd_write_reg(0xD31E,0x02);
        lcd_write_reg(0xD31F,0x67);
        lcd_write_reg(0xD320,0x02);
        lcd_write_reg(0xD321,0x84);
        lcd_write_reg(0xD322,0x02);
        lcd_write_reg(0xD323,0xA4);
        lcd_write_reg(0xD324,0x02);
        lcd_write_reg(0xD325,0xB7);
        lcd_write_reg(0xD326,0x02);
        lcd_write_reg(0xD327,0xCF);
        lcd_write_reg(0xD328,0x02);
        lcd_write_reg(0xD329,0xDE);
        lcd_write_reg(0xD32A,0x02);
        lcd_write_reg(0xD32B,0xF2);
        lcd_write_reg(0xD32C,0x02);
        lcd_write_reg(0xD32D,0xFE);
        lcd_write_reg(0xD32E,0x03);
        lcd_write_reg(0xD32F,0x10);
        lcd_write_reg(0xD330,0x03);
        lcd_write_reg(0xD331,0x33);
        lcd_write_reg(0xD332,0x03);
        lcd_write_reg(0xD333,0x6D);
        lcd_write_reg(0xD400,0x00);
        lcd_write_reg(0xD401,0x33);
        lcd_write_reg(0xD402,0x00);
        lcd_write_reg(0xD403,0x34);
        lcd_write_reg(0xD404,0x00);
        lcd_write_reg(0xD405,0x3A);
        lcd_write_reg(0xD406,0x00);
        lcd_write_reg(0xD407,0x4A);
        lcd_write_reg(0xD408,0x00);
        lcd_write_reg(0xD409,0x5C);
        lcd_write_reg(0xD40A,0x00);
        lcd_write_reg(0xD40B,0x81);

        lcd_write_reg(0xD40C,0x00);
        lcd_write_reg(0xD40D,0xA6);
        lcd_write_reg(0xD40E,0x00);
        lcd_write_reg(0xD40F,0xE5);
        lcd_write_reg(0xD410,0x01);
        lcd_write_reg(0xD411,0x13);
        lcd_write_reg(0xD412,0x01);
        lcd_write_reg(0xD413,0x54);
        lcd_write_reg(0xD414,0x01);
        lcd_write_reg(0xD415,0x82);
        lcd_write_reg(0xD416,0x01);
        lcd_write_reg(0xD417,0xCA);
        lcd_write_reg(0xD418,0x02);
        lcd_write_reg(0xD419,0x00);
        lcd_write_reg(0xD41A,0x02);
        lcd_write_reg(0xD41B,0x01);
        lcd_write_reg(0xD41C,0x02);
        lcd_write_reg(0xD41D,0x34);
        lcd_write_reg(0xD41E,0x02);
        lcd_write_reg(0xD41F,0x67);
        lcd_write_reg(0xD420,0x02);
        lcd_write_reg(0xD421,0x84);
        lcd_write_reg(0xD422,0x02);
        lcd_write_reg(0xD423,0xA4);
        lcd_write_reg(0xD424,0x02);
        lcd_write_reg(0xD425,0xB7);
        lcd_write_reg(0xD426,0x02);
        lcd_write_reg(0xD427,0xCF);
        lcd_write_reg(0xD428,0x02);
        lcd_write_reg(0xD429,0xDE);
        lcd_write_reg(0xD42A,0x02);
        lcd_write_reg(0xD42B,0xF2);
        lcd_write_reg(0xD42C,0x02);
        lcd_write_reg(0xD42D,0xFE);
        lcd_write_reg(0xD42E,0x03);
        lcd_write_reg(0xD42F,0x10);
        lcd_write_reg(0xD430,0x03);
        lcd_write_reg(0xD431,0x33);
        lcd_write_reg(0xD432,0x03);
        lcd_write_reg(0xD433,0x6D);
        lcd_write_reg(0xD500,0x00);
        lcd_write_reg(0xD501,0x33);
        lcd_write_reg(0xD502,0x00);
        lcd_write_reg(0xD503,0x34);
        lcd_write_reg(0xD504,0x00);
        lcd_write_reg(0xD505,0x3A);
        lcd_write_reg(0xD506,0x00);
        lcd_write_reg(0xD507,0x4A);
        lcd_write_reg(0xD508,0x00);
        lcd_write_reg(0xD509,0x5C);
        lcd_write_reg(0xD50A,0x00);
        lcd_write_reg(0xD50B,0x81);

        lcd_write_reg(0xD50C,0x00);
        lcd_write_reg(0xD50D,0xA6);
        lcd_write_reg(0xD50E,0x00);
        lcd_write_reg(0xD50F,0xE5);
        lcd_write_reg(0xD510,0x01);
        lcd_write_reg(0xD511,0x13);
        lcd_write_reg(0xD512,0x01);
        lcd_write_reg(0xD513,0x54);
        lcd_write_reg(0xD514,0x01);
        lcd_write_reg(0xD515,0x82);
        lcd_write_reg(0xD516,0x01);
        lcd_write_reg(0xD517,0xCA);
        lcd_write_reg(0xD518,0x02);
        lcd_write_reg(0xD519,0x00);
        lcd_write_reg(0xD51A,0x02);
        lcd_write_reg(0xD51B,0x01);
        lcd_write_reg(0xD51C,0x02);
        lcd_write_reg(0xD51D,0x34);
        lcd_write_reg(0xD51E,0x02);
        lcd_write_reg(0xD51F,0x67);
        lcd_write_reg(0xD520,0x02);
        lcd_write_reg(0xD521,0x84);
        lcd_write_reg(0xD522,0x02);
        lcd_write_reg(0xD523,0xA4);
        lcd_write_reg(0xD524,0x02);
        lcd_write_reg(0xD525,0xB7);
        lcd_write_reg(0xD526,0x02);
        lcd_write_reg(0xD527,0xCF);
        lcd_write_reg(0xD528,0x02);
        lcd_write_reg(0xD529,0xDE);
        lcd_write_reg(0xD52A,0x02);
        lcd_write_reg(0xD52B,0xF2);
        lcd_write_reg(0xD52C,0x02);
        lcd_write_reg(0xD52D,0xFE);
        lcd_write_reg(0xD52E,0x03);
        lcd_write_reg(0xD52F,0x10);
        lcd_write_reg(0xD530,0x03);
        lcd_write_reg(0xD531,0x33);
        lcd_write_reg(0xD532,0x03);
        lcd_write_reg(0xD533,0x6D);
        lcd_write_reg(0xD600,0x00);
        lcd_write_reg(0xD601,0x33);
        lcd_write_reg(0xD602,0x00);
        lcd_write_reg(0xD603,0x34);
        lcd_write_reg(0xD604,0x00);
        lcd_write_reg(0xD605,0x3A);
        lcd_write_reg(0xD606,0x00);
        lcd_write_reg(0xD607,0x4A);
        lcd_write_reg(0xD608,0x00);
        lcd_write_reg(0xD609,0x5C);
        lcd_write_reg(0xD60A,0x00);
        lcd_write_reg(0xD60B,0x81);

        lcd_write_reg(0xD60C,0x00);
        lcd_write_reg(0xD60D,0xA6);
        lcd_write_reg(0xD60E,0x00);
        lcd_write_reg(0xD60F,0xE5);
        lcd_write_reg(0xD610,0x01);
        lcd_write_reg(0xD611,0x13);
        lcd_write_reg(0xD612,0x01);
        lcd_write_reg(0xD613,0x54);
        lcd_write_reg(0xD614,0x01);
        lcd_write_reg(0xD615,0x82);
        lcd_write_reg(0xD616,0x01);
        lcd_write_reg(0xD617,0xCA);
        lcd_write_reg(0xD618,0x02);
        lcd_write_reg(0xD619,0x00);
        lcd_write_reg(0xD61A,0x02);
        lcd_write_reg(0xD61B,0x01);
        lcd_write_reg(0xD61C,0x02);
        lcd_write_reg(0xD61D,0x34);
        lcd_write_reg(0xD61E,0x02);
        lcd_write_reg(0xD61F,0x67);
        lcd_write_reg(0xD620,0x02);
        lcd_write_reg(0xD621,0x84);
        lcd_write_reg(0xD622,0x02);
        lcd_write_reg(0xD623,0xA4);
        lcd_write_reg(0xD624,0x02);
        lcd_write_reg(0xD625,0xB7);
        lcd_write_reg(0xD626,0x02);
        lcd_write_reg(0xD627,0xCF);
        lcd_write_reg(0xD628,0x02);
        lcd_write_reg(0xD629,0xDE);
        lcd_write_reg(0xD62A,0x02);
        lcd_write_reg(0xD62B,0xF2);
        lcd_write_reg(0xD62C,0x02);
        lcd_write_reg(0xD62D,0xFE);
        lcd_write_reg(0xD62E,0x03);
        lcd_write_reg(0xD62F,0x10);
        lcd_write_reg(0xD630,0x03);
        lcd_write_reg(0xD631,0x33);
        lcd_write_reg(0xD632,0x03);
        lcd_write_reg(0xD633,0x6D);
        
        lcd_write_reg(0xF000,0x55);
        lcd_write_reg(0xF001,0xAA);
        lcd_write_reg(0xF002,0x52);
        lcd_write_reg(0xF003,0x08);
        lcd_write_reg(0xF004,0x00);
       
        lcd_write_reg(0xB100, 0xCC);
        lcd_write_reg(0xB101, 0x00);
        
        lcd_write_reg(0xB600,0x05);
       
        lcd_write_reg(0xB700,0x70);
        lcd_write_reg(0xB701,0x70);
       
        lcd_write_reg(0xB800,0x01);
        lcd_write_reg(0xB801,0x03);
        lcd_write_reg(0xB802,0x03);
        lcd_write_reg(0xB803,0x03);
       
        lcd_write_reg(0xBC00,0x02);
        lcd_write_reg(0xBC01,0x00);
        lcd_write_reg(0xBC02,0x00);
       
        lcd_write_reg(0xC900,0xD0);
        lcd_write_reg(0xC901,0x02);
        lcd_write_reg(0xC902,0x50);
        lcd_write_reg(0xC903,0x50);
        lcd_write_reg(0xC904,0x50);
        lcd_write_reg(0x3500,0x00);
        lcd_write_reg(0x3A00,0x55);
        lcd_write_cmd(0x1100);
        HAL_Delay(1);
        lcd_write_cmd(0x2900);
    } else {
        LCD_LOG("Not find LCD Init Code, LCD ID is:%#x\r\n", lcd_params.lcd_id);
	}
    
    LCD_LOG("LCD Init Success\r\n");
    
    /* 设置LCD显示方向，竖屏：0，横屏：1*/
	lcd_set_display_drection(LCD_DEFAULT_DISPLAY_DIR);
    
    /* 打开显示 */
    lcd_display_on();
    
    /* LCD清屏 */
	//lcd_clear(LCD_DEFAULT_CLEAR_COLOR);

    /* 打开背光 */
    lcd_backlight_ctrl(LCD_BACKLIGHT_ON);
    
    return;
}

/**
 * @brief    LCD打点函数
 * @param    x_pos x方向坐标
 * @param    y_pos y方向坐标
 * @retval   none
*/
void lcd_draw_point(uint16_t x_pos, uint16_t y_pos, uint16_t color)
{
    if (x_pos > lcd_params.lcd_width || y_pos > lcd_params.lcd_height) {
        return;
    }
    
    lcd_set_cursor(x_pos, y_pos);
    lcd_write_ram_start();
    lcd_write_ram(color);
}

/**
 * @brief    LCD设置窗口
 * @param    x_pos_start x方向起始坐标
 * @param    y_pos_start y方向起始坐标
 * @param    width       窗口宽度
 * @param    height      窗口高度
 * @retval   none
 * @note     此函数执行完，坐标在窗口左上角
*/
void lcd_set_window(uint16_t x_pos_start, uint16_t y_pos_start, uint16_t x_pos_end, uint16_t y_pos_end)
{
    
    if(lcd_params.lcd_id == 0x9341 || lcd_params.lcd_id == 0x5310) {
		lcd_write_cmd(lcd_params.set_x_cmd); 
		lcd_write_data(x_pos_start >> 8); 
		lcd_write_data(x_pos_start & 0xFF);	 
		lcd_write_data(x_pos_end >> 8); 
		lcd_write_data(x_pos_end & 0xFF);  
		lcd_write_cmd(lcd_params.set_y_cmd); 
		lcd_write_data(y_pos_start >> 8); 
		lcd_write_data(y_pos_start & 0xFF); 
		lcd_write_data(y_pos_end >> 8); 
		lcd_write_data(y_pos_end & 0xFF); 
	} else if (lcd_params.lcd_id == 0x5510) {
		lcd_write_cmd(lcd_params.set_x_cmd); 
        lcd_write_data(x_pos_start >> 8); 
		lcd_write_cmd(lcd_params.set_x_cmd + 1);
        lcd_write_data(x_pos_start & 0xFF);	  
		lcd_write_cmd(lcd_params.set_x_cmd + 2);
        lcd_write_data(x_pos_end >> 8);   
		lcd_write_cmd(lcd_params.set_x_cmd + 3);
        lcd_write_data(x_pos_end & 0xFF);   
		lcd_write_cmd(lcd_params.set_y_cmd);
        lcd_write_data(y_pos_start >> 8);   
		lcd_write_cmd(lcd_params.set_y_cmd + 1);
        lcd_write_data(y_pos_start&0xFF);  
		lcd_write_cmd(lcd_params.set_y_cmd + 2);
        lcd_write_data(y_pos_end >> 8);   
		lcd_write_cmd(lcd_params.set_y_cmd + 3);
        lcd_write_data(y_pos_end & 0xFF);  
	}
}

/**
 * @brief   LCD画线函数
 * @param   x1 x方向起始坐标
 * @param   x2 x方向终止坐标
 * @param   y1 y方向起始坐标
 * @param   y2 y方向终止坐标
 * @return  none
 */
void lcd_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t	i = 0;
	int16_t		delta_x = 0, delta_y = 0;
	int8_t		incx = 0, incy = 0;
	uint16_t	distance = 0;
	uint16_t    t = 0;
	uint16_t	x = 0, y = 0;
	uint16_t 	x_temp = 0, y_temp = 0;
    
    if(y1 == y2)
    {
     /* 快速画水平线 */
        lcd_set_window(x1, y1, x2, y2);
        lcd_write_ram_start();
        for(i = 0; i < x2 - x1; i++)
        {
            lcd_write_ram(color);
        }

        return;
    }
	else
	{
		/* 画斜线（Bresenham算法） */
		/* 计算两点之间在x和y方向的间距，得到画笔在x和y方向的步进值 */
		delta_x = x2 - x1;
		delta_y = y2 - y1;
		if(delta_x > 0)
		{
			//斜线(从左到右)
			incx = 1;
		}
		else if(delta_x == 0)
		{
			//垂直斜线(竖线)
			incx = 0;
		}
		else
		{
			//斜线(从右到左)
			incx = -1;
			delta_x = -delta_x;
		}
		if(delta_y > 0)
		{
			//斜线(从左到右)
			incy = 1;
		}
		else if(delta_y == 0)
		{
			//水平斜线(水平线)
			incy = 0;
		}
		else
		{
			//斜线(从右到左)
			incy = -1;
			delta_y = -delta_y;
		}			
		
		/* 计算画笔打点距离(取两个间距中的最大值) */
		if(delta_x > delta_y)
		{
			distance = delta_x;
		}
		else
		{
			distance = delta_y;
		}
		
		/* 开始打点 */
		x = x1;
		y = y1;
		//第一个点无效，所以t的次数加一
		for(t = 0; t <= distance + 1;t++)
		{
			lcd_draw_point(x, y, color);
		
			/* 判断离实际值最近的像素点 */
			x_temp += delta_x;	
			if(x_temp > distance)
			{
				//x方向越界，减去距离值，为下一次检测做准备
				x_temp -= distance;		
				//在x方向递增打点
				x += incx;
					
			}
			y_temp += delta_y;
			if(y_temp > distance)
			{
				//y方向越界，减去距离值，为下一次检测做准备
				y_temp -= distance;
				//在y方向递增打点
				y += incy;
			}
		}
	}
}

/**
 * @breif	LCD画矩形
 * @param   x1 x方向起始坐标
 * @param   x2 x方向终止坐标
 * @param   y1 y方向起始坐标
 * @param   y2 y方向终止坐标
 * @param	color 颜色
 * @retval	none
 */
void lcd_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	lcd_draw_line(x1,y1,x2,y1,color);
	lcd_draw_line(x1,y1,x1,y2,color);
	lcd_draw_line(x1,y2,x2,y2,color);
	lcd_draw_line(x2,y1,x2,y2,color);
}

/**
 * @breif	LCD画圆函数
 * @param   x x方向坐标
 * @param	y 方向坐标
 * @param   r 半径
 * @param	color 颜色
 * @retval	none
 */
void lcd_draw_circle(uint16_t x, uint16_t y, uint16_t r, uint16_t color)
{
	/* Bresenham画圆算法 */
	int16_t a = 0, b = r;
    int16_t d = 3 - (r << 1);		//算法决策参数
		
	/* 如果圆在屏幕可见区域外，直接退出 */
    if (x - r < 0 || x + r > lcd_params.lcd_width || y - r < 0 || y + r > lcd_params.lcd_height) {
        return;
    }
		
	/* 开始画圆 */
    while(a <= b)
    {
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

        if(d < 0)
			d += 4 * a + 6;
        else
        {
            d += 10 + 4 * (a - b);
            b--;
        }

        lcd_draw_point(x + a, y + b, color);
    }
}

/**
 * @breif	LCD填充一个矩形区域
 * @param   x1 x方向起始坐标
 * @param   x2 x方向终止坐标
 * @param   y1 y方向起始坐标
 * @param   y2 y方向终止坐标
 * @param	color 颜色
 * @retval	none
 */
void lcd_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t i, j;
    uint32_t xlen = 0;
    
    xlen = x2 - x1 + 1;
    for (i = y1; i <= y2; i++) {
        lcd_set_cursor(x1, i);
        lcd_write_ram_start();
        for (j = 0; j < xlen; j++) {
            lcd_write_ram(color);
        }
    }
}

/**
 * @breif	LCD使用给定缓冲填充一个矩形区域
 * @param   x1 x方向起始坐标
 * @param   x2 x方向终止坐标
 * @param   y1 y方向起始坐标
 * @param   y2 y方向终止坐标
 * @param	color_buf 数据缓冲区
 * @retval	none
 */
void lcd_fill_with_buffer(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *color_buf)
{
    uint32_t index = 0;      
	uint32_t totalpoint = 0;
    uint16_t width = x2 - x1 + 1;
    uint16_t height = y2 - y1 + 1;
    
    /* 计算得到总点数 */
	totalpoint = width * height;
    
	lcd_set_window(x1, y1, x2, y2);
	lcd_write_ram_start();
    
    /* 写入数据到GRAM */
	for (index = 0; index < totalpoint; index++) {
		lcd_write_ram(*color_buf++);
	}
}
