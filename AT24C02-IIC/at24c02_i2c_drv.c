/**
 * @Copyright 			(c) 2019,mculover666 All rights reserved	
 * @filename  			at24c02_i2c_drv.c
 * @breif				Drive AT24C02(EEPROM) based on iic1 commucation interface
 * @version
 *            			v1.0    完成基本驱动程序              mculover666    2019/7/15
 * @note            	移植说明（非常重要）：	
 *						1. 修改at24c02_i2c_drv.h文件中的AT24C02读写地址即可；
 * 						2. 此驱动程序需要STM32CubeMX生成的I2C初始化文件i2c.h和i2c.c支持。
 */

#include "at24c02_i2c_drv.h"
#include "i2c.h"

/**
 * @brief		AT24C02任意地址写一个字节数据
 * @param		addr —— 写数据的地址（0-255）
 * @param		dat  —— 存放写入数据的地址
 * @retval		成功 —— HAL_OK
*/
uint8_t At24c02_Write_Byte(uint16_t addr, uint8_t* dat)
{
		return HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, dat, 1, 0xFFFF);
}
/**
 * @brief		AT24C02任意地址读一个字节数据
 * @param		addr —— 读数据的地址（0-255）
 * @param		read_buf —— 存放读取数据的地址
 * @retval		成功 —— HAL_OK
*/
uint8_t At24c02_Read_Byte(uint16_t addr, uint8_t* read_buf)
{
		return HAL_I2C_Mem_Read(&hi2c1, AT24C02_ADDR_READ, addr, I2C_MEMADD_SIZE_8BIT, read_buf, 1, 0xFFFF);
}
/**
 * @brief		AT24C02任意地址连续写多个字节数据
 * @param		addr —— 写数据的地址（0-255）
 * @param		dat  —— 存放写入数据的地址
 * @retval		成功 —— HAL_OK
*/
uint8_t At24c02_Write_Amount_Byte(uint16_t addr, uint8_t* dat, uint16_t size)
{
		uint8_t i = 0;
		uint16_t cnt = 0;		//写入字节计数
		
		/* 对于起始地址，有两种情况，分别判断 */
		if(0 == addr % 8 )
		{
			/* 起始地址刚好是页开始地址 */
			
			/* 对于写入的字节数，有两种情况，分别判断 */
			if(size <= 8)
			{
				//写入的字节数不大于一页，直接写入
				return HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, dat, size, 0xFFFF);
			}
			else
			{
				//写入的字节数大于一页，先将整页循环写入
				for(i = 0;i < size/8; i++)
				{
						HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, &dat[cnt], 8, 0xFFFF);
						addr += 8;
						cnt += 8;
				}
				//将剩余的字节写入
				return HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, &dat[cnt], size - cnt, 0xFFFF);
			}
		}
		else
		{
			/* 起始地址偏离页开始地址 */
			/* 对于写入的字节数，有两种情况，分别判断 */
			if(size <= (8 - addr%8))
			{
				/* 在该页可以写完 */
				return HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, dat, size, 0xFFFF);
			}
			else
			{
				/* 该页写不完 */
				//先将该页写完
				cnt += 8 - addr%8;
				HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, dat, cnt, 0xFFFF);
				addr += cnt;
				
				//循环写整页数据
				for(i = 0;i < (size - cnt)/8; i++)
				{
						HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, &dat[cnt], 8, 0xFFFF);
						addr += 8;
						cnt += 8;
				}
				
				//将剩下的字节写入
				return HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, &dat[cnt], size - cnt, 0xFFFF);
			}			
		}
}
/**
 * @brief		AT24C02任意地址连续读多个字节数据
 * @param		addr —— 读数据的地址（0-255）
 * @param		dat  —— 存放读出数据的地址
 * @retval		成功 —— HAL_OK
*/
uint8_t At24c02_Read_Amount_Byte(uint16_t addr, uint8_t* recv_buf, uint16_t size)
{
	return HAL_I2C_Mem_Read(&hi2c1, AT24C02_ADDR_READ, addr, I2C_MEMADD_SIZE_8BIT, recv_buf, size, 0xFFFF);
}
