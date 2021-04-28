/**
 * @Copyright   (c) 2019,mculover666 All rights reserved	
 * @filename    at24C02_i2c_drv.c
 * @breif       Drive AT24C02(EEPROM) based on stm32 iic peripheral
 * @changelog   v1.0    mculover666     2019/7/15
 *              v1.1    mculover666     2021/4/15   (add WRITE_CYCLE_TIME_MS)
 *              v1.2    mculover666     2021/4/28   (add value operation)
 */

#include "AT24C02_i2c_drv.h"
#include "i2c.h"

/**
 * @brief       write a byte to any address
 * @param[in]	addr    address to write  
 * @param[in]   data    the pointer of data to write
 * @return		errcode
 * @retval      0       success
 * @retval      -1      fail
*/
int AT24C02_Write_Byte(uint16_t addr, uint8_t* dat)
{
    int ret;
    
    HAL_Delay(WRITE_CYCLE_TIME_MS);
    
    ret = HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, dat, 1, 100);
    
    return ret == HAL_OK ? 0 : -1;
}

/**
 * @brief       read a byte from any address
 * @param[in]	addr        address to rad  
 * @param[out]  read_buf    the pointer of buffer to save read data
 * @return		errcode
 * @retval      0       success
 * @retval      -1      fail
*/
int AT24C02_Read_Byte(uint16_t addr, uint8_t* read_buf)
{
    int ret;

    HAL_Delay(WRITE_CYCLE_TIME_MS);
    
    ret = HAL_I2C_Mem_Read(&hi2c1, AT24C02_ADDR_READ, addr, I2C_MEMADD_SIZE_8BIT, read_buf, 1, 100);
    
    return ret == HAL_OK ? 0 : -1;
}

/**
 * @brief       write amount bytes to any address
 * @param[in]	addr    address to write
 * @param[in]   data    the pointer of data buffer to write
 * @return		errcode
 * @retval      0       success
 * @retval      else      fail
*/
int AT24C02_Write_Amount_Byte(uint16_t addr, uint8_t* dat, uint16_t size)
{
    int ret;
    uint16_t i = 0;
    uint16_t cnt = 0;
    uint16_t page = 0;
    
    if (0 == addr % 8) {    // whether the addr is the starting address of the page
        if (size <= 8) {    //whether the number of bytes to be written is less than one page
            HAL_Delay(WRITE_CYCLE_TIME_MS);
            ret = HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, dat, size, 100);
            if (ret != HAL_OK) {
                return -1;
            }
        } else {
            page = size / 8;
            for (i = 0;i < page; i++) {
                HAL_Delay(WRITE_CYCLE_TIME_MS);
                ret = HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, &dat[cnt], 8, 100);
                if (ret != HAL_OK) {
                    return -2;
                }
                addr += 8;
                cnt += 8;
            }
            if (cnt < size) {     //write remain bytes
                HAL_Delay(WRITE_CYCLE_TIME_MS);
                ret = HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, &dat[cnt], size - cnt, 100);
                if (ret != HAL_OK) {
                    return -3;
                }
            }
        }
    } else {    //the address deviates from the starting page address
        if (size <= (8 - addr % 8)) {     //whether we can finish writing on this page
            HAL_Delay(WRITE_CYCLE_TIME_MS);
            ret = HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, dat, size, 100);
            if (ret != HAL_OK) {
                return -4;
            }
        } else {    //can't finish on this page
            //finish the page first
            cnt += 8 - addr % 8;
            HAL_Delay(WRITE_CYCLE_TIME_MS);
            ret = HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, dat, cnt, 100);
            if (ret != HAL_OK) {
                return -5;
            }
            addr += cnt;
            
            //loops to write the entire page of data
            page = (size - cnt) / 8;
            for (i = 0;i < page; i++) { 
                HAL_Delay(WRITE_CYCLE_TIME_MS);
                ret = HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, &dat[cnt], 8, 100);
                if (ret != HAL_OK) {
                    return -6;
                }
                addr += 8;
                cnt += 8;
            }
            if (cnt < size) {     //write remain bytes
                HAL_Delay(WRITE_CYCLE_TIME_MS);
                ret = HAL_I2C_Mem_Write(&hi2c1, AT24C02_ADDR_WRITE, addr, I2C_MEMADD_SIZE_8BIT, &dat[cnt], size - cnt, 100);
                if (ret != HAL_OK) {
                    return -7;
                }
            }
        }			
    }
    
    return 0;
}

/**
 * @brief       read amount bytes from any address
 * @param[in]	addr        address to read  
 * @param[out]  read_buf    the pointer of buffer to save read data
 * @return		errcode
 * @retval      0       success
 * @retval      -1      fail
*/
int AT24C02_Read_Amount_Byte(uint16_t addr, uint8_t* recv_buf, uint16_t size)
{
    int ret;
    
    HAL_Delay(WRITE_CYCLE_TIME_MS);
    
	ret = HAL_I2C_Mem_Read(&hi2c1, AT24C02_ADDR_READ, addr, I2C_MEMADD_SIZE_8BIT, recv_buf, size, 100);
    
    return ret == HAL_OK ? 0 : -1;
}

/**
 * @brief       write value to any address
 * @param[in]	addr    address to write
 * @param[in]   ptr     the pointer of value to write
 * @return		errcode
 * @retval      0       success
 * @retval      else      fail
*/
int AT24C02_Write_Value(uint16_t addr, void *ptr, uint16_t size)
{
    int ret;
    
    if (!ptr) {
        return -1;
    }
  
    ret = AT24C02_Write_Amount_Byte(addr, (uint8_t*)ptr, size);
    
    if (ret < 0) {
        return ret;
    }
    
    return 0;
}

/**
 * @brief       read value from any address
 * @param[in]	addr    address to write
 * @param[in]   ptr     the pointer of value to read
 * @return		errcode
 * @retval      0       success
 * @retval      else      fail
*/
int AT24C02_Read_Value(uint16_t addr, void *ptr, uint16_t size)
{
    int ret;
    
    if (!ptr) {
        return -1;
    }
  
    ret = AT24C02_Read_Amount_Byte(addr, (uint8_t*)ptr, size);
    
    if (ret < 0) {
        return ret;
    }
    
    return 0;
}
