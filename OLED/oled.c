#include "oled.h"
#include "oledfont.h"

//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 			   

/**
 * @brief	OLED写入命令
 * @param cmd - 待写入命令
 * @note	移植时，请使用自己的底层API实现 
*/
static void OLED_Write_Cmd(uint8_t cmd)
{
	uint8_t buf[2];
	buf[0] = 0x00;	//control byte
	buf[1] = cmd;
	
	//使用HAL库的API实现
	HAL_I2C_Master_Transmit(&hi2c1, 0x78, buf, 2, 0xFFFF);
}
/**
 * @brief	OLED写入数据
 * @param cmd - 待写入数据
 * @note	移植时，请使用自己的底层API实现 
*/
static void OLED_Write_Dat(uint8_t dat)
{
	uint8_t buf[2];
	buf[0] = 0x40; //control byte
	buf[1] = dat;
	
	//使用HAL库的API实现
	HAL_I2C_Master_Transmit(&hi2c1, 0x78, buf, 2, 0xFFFF);
}

/**
 * @brief	OLED设置显示位置
 * @param x - X方向位置
 * @param y - Y方向位置
*/
void OLED_Set_Pos(uint8_t x, uint8_t y)
{ 	
	OLED_Write_Cmd(0xb0+y);
	OLED_Write_Cmd(((x&0xf0)>>4)|0x10);
	OLED_Write_Cmd((x&0x0f)|0x01);
}   	  
/**
 * @brief	OLED开启显示
*/ 
void OLED_Display_On(void)
{
	OLED_Write_Cmd(0X8D);  //SET DCDC命令
	OLED_Write_Cmd(0X14);  //DCDC ON
	OLED_Write_Cmd(0XAF);  //DISPLAY ON
}
/**
 * @brief	OLED关闭显示
*/   
void OLED_Display_Off(void)
{
	OLED_Write_Cmd(0X8D);  //SET DCDC命令
	OLED_Write_Cmd(0X10);  //DCDC OFF
	OLED_Write_Cmd(0XAE);  //DISPLAY OFF
}		   			 
/**
 * @brief	OLED清屏函数（清屏之后屏幕全为黑色）
*/ 
void OLED_Clear(void)
{  
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_Write_Cmd(0xb0+i);    //设置页地址（0~7）
		OLED_Write_Cmd(0x00);      //设置显示位置—列低地址
		OLED_Write_Cmd(0x10);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)
		{
			OLED_Write_Dat(0);
		}			
	}
}
/**
 * @brief	OLED显示全开（所有像素点全亮）
*/
void OLED_On(void)
{  
	uint8_t i,n;
	for(i=0;i<8;i++)
	{  
		OLED_Write_Cmd(0xb0+i);    //设置页地址（0~7）
		OLED_Write_Cmd(0x00);      //设置显示位置—列低地址
		OLED_Write_Cmd(0x10);      //设置显示位置—列高地址   
		for(n=0;n<128;n++)
		{
			OLED_Write_Dat(1);
		}			
	}
}
/**
 * @brief	在指定位置显示一个ASCII字符
 * @param x - 0 - 127
 * @param y - 0 - 7
 * @param chr  - 待显示的ASCII字符
 * @param size - ASCII字符大小
 * 				字符大小有12(6*8)/16(8*16)两种大小
*/
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size)
{   
	uint8_t c=0,i=0;
	
	c = chr-' ';	
	if(x > 128-1)
	{
		x=0;
		y++;
	}
	
	if(size ==16)
	{
		OLED_Set_Pos(x,y);	
		for(i=0;i<8;i++)
		{
			OLED_Write_Dat(F8X16[c*16+i]);
		}
		OLED_Set_Pos(x,y+1);
		for(i=0;i<8;i++)
		{
			OLED_Write_Dat(F8X16[c*16+i+8]);
		}
	}
	else
	{	
		OLED_Set_Pos(x,y);
		for(i=0;i<6;i++)
		{
			OLED_Write_Dat(F6x8[c][i]);
		}
	}
}
/**
 * @brief	OLED 专用pow函数
 * @param m - 底数
 * @param n - 指数
*/
static uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}	
/**
 * @brief	在指定位置显示一个整数
 * @param x - 0 - 127
 * @param y - 0 - 7
 * @param num - 待显示的整数(0-4294967295)
 * @param	len - 数字的位数 
 * @param size - ASCII字符大小
 * 				字符大小有12(6*8)/16(8*16)两种大小
*/
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size); 
	}
} 
/**
 * @brief	在指定位置显示一个字符串
 * @param x - 0 - 127
 * @param y - 0 - 7
 * @param chr - 待显示的字符串指针
 * @param size - ASCII字符大小
 * 				字符大小有12(6*8)/16(8*16)两种大小
*/
void OLED_ShowString(uint8_t x,uint8_t y,char *chr,uint8_t size)
{
	uint8_t j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar(x,y,chr[j],size);
			x+=8;
			if(x>120){x=0;y+=2;}
			j++;
	}
}
/**
 * @brief	在指定位置显示一个汉字
 * @param x  - 0 - 127
 * @param y  - 0 - 7
 * @param no - 汉字在中文字库数组中的索引（下标）
 * @note	中文字库在oledfont.h文件中的Hzk数组中，需要提前使用软件对汉字取模
*/
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no)
{      			    
	uint8_t t,adder=0;
	OLED_Set_Pos(x,y);	
    for(t=0;t<16;t++)
		{
				OLED_Write_Dat(Hzk[2*no][t]);
				adder+=1;
     }	
		OLED_Set_Pos(x,y+1);	
    for(t=0;t<16;t++)
			{	
				OLED_Write_Dat(Hzk[2*no+1][t]);
				adder+=1;
      }					
}
/**
 * @brief	在指定位置显示一幅图片
 * @param x1,x2  - 0 - 127
 * @param y1,y2  - 0 - 7(8表示全屏显示)
 * @param BMP - 图片数组地址
 * @note	图像数组BMP存放在bmp.h文件中
*/
void OLED_DrawBMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,uint8_t BMP[])
{ 	
 uint16_t j=0;
 uint8_t x,y;
  
  if(y1%8==0)
	{
		y=y1/8;
	}		
  else
	{
		y=y1/8+1;
	}
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
	  {      
	    	OLED_Write_Dat(BMP[j++]);	    	
	  }
	}
} 

/**
 * @brief	OLED初始化
*/			    
void OLED_Init(void)
{ 	

	HAL_Delay(500);

	OLED_Write_Cmd(0xAE);//--display off
	OLED_Write_Cmd(0x00);//---set low column address
	OLED_Write_Cmd(0x10);//---set high column address
	OLED_Write_Cmd(0x40);//--set start line address  
	OLED_Write_Cmd(0x81); // contract control
	OLED_Write_Cmd(0xFF);//--128   
	OLED_Write_Cmd(0xA1);//set segment remap 
	OLED_Write_Cmd(0xC8);//Com scan direction
	OLED_Write_Cmd(0xA6);//--normal / reverse
	OLED_Write_Cmd(0xA8);//--set multiplex ratio(1 to 64)
	OLED_Write_Cmd(0x3F);//--1/32 duty
	OLED_Write_Cmd(0xD3);//-set display offset
	OLED_Write_Cmd(0x00);//
	OLED_Write_Cmd(0xD5);//set osc division
	OLED_Write_Cmd(0x80);
	OLED_Write_Cmd(0xD9);//Set Pre-Charge Period
	OLED_Write_Cmd(0xF1);//
	OLED_Write_Cmd(0xDA);//set com pin configuartion
	OLED_Write_Cmd(0x12);//
	OLED_Write_Cmd(0xDB);//set Vcomh
	OLED_Write_Cmd(0x40);//
	OLED_Write_Cmd(0x20);
	OLED_Write_Cmd(0x02);
	OLED_Write_Cmd(0x8D);//set charge pump enable
	OLED_Write_Cmd(0x14);//
	OLED_Write_Cmd(0xA4);
	OLED_Write_Cmd(0xA6);
	OLED_Write_Cmd(0xAF);//--turn on oled panel
	
	OLED_Clear();
	OLED_Set_Pos(0,0);
}
