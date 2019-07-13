/**
 * @Copyright 			   (c) 2019,mculover666 All rights reserved	
 * @filename  			   main.c
 * @breif				       测试LCD驱动程序
 * @version
 *            			   v1.0    完成基本驱动程序，可以刷屏              mculover666    2019/7/10
*                      v1.1    添加打点、画线、画矩形、画圆算法实现     mculover666    2019/7/12
*                      v1.2    添加显示英文ASCII字符和字符串           mculover666    2019/7/12
*                      v1.3    添加绘制六芒星函数（基于画线函数）       mculover666    2019/7/12
*                      v1.4    添加显示图片函数                       mculover666     2019/7/13   
*                      v2.0    使用宏开关控制字符显示和图片显示         mculover666    2019/7/13  
 * @note                移植说明（非常重要）：
 *                      1. LCD_SPI_Send是LCD的底层发送函数，如果是不同的芯片或者SPI接口，使用CubeMX生成初始化代码，
 *                         先修改此"lcd_spi2_drv.h"的LCD控制引脚宏定义，
 *                         然后修改LCD_SPI_Send中的内容即可；
 *                      2. 如果使用的是ST7789V2液晶控制器，但是不同分辨率的屏幕，修改"lcd_spi2_drv.h"中的LCD_Width和LCD_Height宏定义即可；
 *                      3. LCD_Buf_Size请勿轻易修改，会影响几乎所有的函数，除非你明确的了解后果；
 *                      4. 此驱动程序需要spi.h和spi.c的支持；
 *                      5. 其余情况不适配此驱动代码。
 *                      6. 使用ASCII字符显示功能和图片显示功能请先在"lcd_spi2_drv.c"打开对应的宏开关;
 *                      7. 由于整数和浮点数情况较多，本驱动程序不包含，请使用sprintf控制格式，然后调用字符串显示函数，敬请谅解。
 */

#include "stm32l4xx_hal.h"
#include "lcd_spi2_drv.h"
//该头文件是图片数据，请在测试图片显示时打开USE_PICTURE_DISPLAY宏开关
#if USE_PICTURE_DISPLAY
#include "bear.h"   
#endif /* USE_PICTURE_DISPLAY */

void SystemClock_Config(void);

int main(void)
{

  HAL_Init();

  SystemClock_Config();

	LCD_Init();

  /*
  //以下9行代码是测试画线、画矩形、画圆使用，如非必要，请勿取消注释
	LCD_Draw_ColorLine(0,120,240,120,RED);		//画水平线
	LCD_Draw_ColorLine(0,0,240,240,BLUE);	  	//画斜线(从左到右，45°)
 	LCD_Draw_ColorLine(0,240,240,0,GREEN);		//画斜线(从右到左，45°)
	LCD_Draw_ColorLine(120,0,120,240,YELLOW);	//画垂直线
	LCD_Draw_ColorLine(180,0,60,240,RED);			//画斜线(从左到右，120°)
	LCD_Draw_ColorLine(60,0,180,240,RED);			//画斜线(从右到左，60°)
	LCD_Draw_ColorLine(0,60,240,180,RED);			//画斜线(从左到右，180°)
	LCD_Draw_ColorLine(0,180,240,60,RED);			//画斜线(从左到右，30°)
	LCD_Draw_ColorRect(60,60,180,180,PINK);		//画矩形
	LCD_Draw_ColorCircle(120,120,85, GBLUE);	//画圆

  */

  /*
  //以下9行测试12/16/24/32四种字符和字符串显示
  LCD_ShowChar(6,12,'B',BLACK,YELLOW,16);
  LCD_ShowChar(14,28,'C',BLACK,GREEN,24);
  LCD_ShowChar(0,0,'A',BLACK,BLUE,12);
  LCD_ShowChar(26,52,'D',BLACK,PINK,32);
  LCD_ShowCharStr(60,240-32-24-24-24,120,"Powerd BY",BLACK,GREEN,24);
  LCD_ShowCharStr(36,240-32-24-24,150,"HUAWEI LiteOS",BLACK,YELLOW,24);
  LCD_ShowCharStr(28,240-32-24,176,"Mculover666",BLACK,BLUE,32);
  LCD_ShowCharStr(12,240-24,240,"Copyright (c) 2019",BLACK,PINK,24);
  LCD_Draw_ColorSixPointStar(150,65,40,RED);
  */

  /*
  //以下2行测试图片显示，需要bear.h头文件的支持
  LCD_Show_Image(0,0,240,240,gImage_bear);
	LCD_ShowCharStr(70,240-24,140,"Starting...",WHITE,BLUE,24);
  
  
   */
	while (1)
  {
    /* 
    //以下代码10行代码是测试刷屏使用 
		LCD_Clear(WHITE);
		LCD_Clear(YELLOW);
		LCD_Clear(BRRED);
		LCD_Clear(PINK);
		LCD_Clear(RED);
		LCD_Clear(BROWN);
		LCD_Clear(GRAY);
		LCD_Clear(GBLUE);
		LCD_Clear(GREEN);
		LCD_Clear(BLUE);
		LCD_Clear(BLACK);
    */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    //Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    //Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
