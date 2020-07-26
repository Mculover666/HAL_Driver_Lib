/**
 * @file    w25q64.c
 * @brief   W25Q64驱动程序
 * @author  Mculover666<http://www.mculover666.cn>
 * @note    此驱动程序基于STM32CubeMX生成的SPI1初始化工程
*/

#include "w25q64.h"

/* 移植相关函数 */

/**
 * @brief    SPI发送指定长度的数据
 * @param    send_buf  —— 发送数据缓冲区首地址
 * @param    size      —— 要发送数据的字节数
 * @retval   成功返回HAL_OK
 */
static HAL_StatusTypeDef SPI_Transmit(uint8_t* send_buf, uint16_t size)
{
    return HAL_SPI_Transmit(&SPI_Handle, send_buf, size, 100);
}

/**
 * @brief   SPI接收指定长度的数据
 * @param   recv_buf  —— 接收数据缓冲区首地址
 * @param   size      —— 要接收数据的字节数
 * @retval  成功返回HAL_OK
 */
static HAL_StatusTypeDef SPI_Receive(uint8_t* recv_buf, uint16_t size)
{
   return HAL_SPI_Receive(&SPI_Handle, recv_buf, size, 100);
}

/**
 * @brief   读取Flash内部的ID
 * @param   none
 * @retval  成功返回device_id
 */
uint16_t W25QXX_ReadID(void)
{
    uint8_t recv_buf[2] = {0};    //recv_buf[0]存放Manufacture ID, recv_buf[1]存放Device ID
    uint16_t device_id = 0;
    uint8_t send_data[4] = {ManufactDeviceID_CMD,0x00,0x00,0x00};   //待发送数据，命令+地址
    
    /* 使能片选 */
    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_RESET);
    
    /* 发送并读取数据 */
    if (HAL_OK == SPI_Transmit(send_data, 4)) {
        if (HAL_OK == SPI_Receive(recv_buf, 2)) {
            device_id = (recv_buf[0] << 8) | recv_buf[1];
        }
    }
    
    /* 取消片选 */
    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_SET);
    
    return device_id;
}

/**
 * @brief     读取W25QXX的状态寄存器，W25Q64一共有2个状态寄存器
 * @param     reg  —— 状态寄存器编号(1~2)
 * @retval    状态寄存器的值
 */
static uint8_t W25QXX_ReadSR(uint8_t reg)
{
    uint8_t result = 0; 
    uint8_t send_buf[4] = {0x00,0x00,0x00,0x00};
    switch(reg)
    {
        case 1:
            send_buf[0] = READ_STATU_REGISTER_1;
        case 2:
            send_buf[0] = READ_STATU_REGISTER_2;
        case 0:
        default:
            send_buf[0] = READ_STATU_REGISTER_1;
    }
    
    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_RESET);

    if (HAL_OK == SPI_Transmit(send_buf, 4)) {
        if (HAL_OK == SPI_Receive(&result, 1)) {
            
             HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_SET);
            return result;
        }
    }
    
     HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_SET);
    
    return 0;
}

/**
 * @brief    阻塞等待Flash处于空闲状态
 * @param   none
 * @retval  none
 */
static void W25QXX_Wait_Busy(void)
{
    while((W25QXX_ReadSR(1) & 0x01) == 0x01); // 等待BUSY位清空
}

/**
 * @brief   读取SPI FLASH数据
 * @param   buffer      —— 数据存储区
 * @param   start_addr  —— 开始读取的地址(最大32bit)
 * @param   nbytes      —— 要读取的字节数(最大65535)
 * @retval  成功返回0，失败返回-1
 */
int W25QXX_Read(uint8_t* buffer, uint32_t start_addr, uint16_t nbytes)
{
    uint8_t cmd = READ_DATA_CMD;
    
    start_addr = start_addr << 8;
    
	W25QXX_Wait_Busy();
    
     /* 使能片选 */
    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_RESET);
    
    SPI_Transmit(&cmd, 1);
    
    if (HAL_OK == SPI_Transmit((uint8_t*)&start_addr, 3)) {
        if (HAL_OK == SPI_Receive(buffer, nbytes)) {
            HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_SET);
            return 0;
        }
    }
    
    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_SET);
    return -1;
}

/**
 * @brief    W25QXX写使能,将S1寄存器的WEL置位
 * @param    none
 * @retval
 */
void W25QXX_Write_Enable(void)
{
    uint8_t cmd= WRITE_ENABLE_CMD;
    
    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_RESET);
    
    SPI_Transmit(&cmd, 1);
    
    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_SET);
    
    W25QXX_Wait_Busy();

}

/**
 * @brief    W25QXX写禁止,将WEL清零
 * @param    none
 * @retval    none
 */
void W25QXX_Write_Disable(void)
{
    uint8_t cmd = WRITE_DISABLE_CMD;

    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_RESET);
    
    SPI_Transmit(&cmd, 1);
    
    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_SET);
    
    W25QXX_Wait_Busy();
}

/**
 * @brief    W25QXX擦除一个扇区
 * @param   sector_addr    —— 扇区地址 根据实际容量设置
 * @retval  none
 * @note    阻塞操作
 */
void W25QXX_Erase_Sector(uint32_t sector_addr)
{
    uint8_t cmd = SECTOR_ERASE_CMD;
    
    sector_addr *= 4096;    //每个块有16个扇区，每个扇区的大小是4KB，需要换算为实际地址
    sector_addr <<= 8;
    
    W25QXX_Write_Enable();  //擦除操作即写入0xFF，需要开启写使能
    W25QXX_Wait_Busy();        //等待写使能完成
   
     /* 使能片选 */
    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_RESET);
    
    SPI_Transmit(&cmd, 1);
    
    SPI_Transmit((uint8_t*)&sector_addr, 3);
    
    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_SET);
    
    W25QXX_Wait_Busy();       //等待扇区擦除完成
}

/**
 * @brief    页写入操作
 * @param    dat —— 要写入的数据缓冲区首地址
 * @param    WriteAddr —— 要写入的地址
 * @param   byte_to_write —— 要写入的字节数（0-256）
 * @retval    none
 */
void W25QXX_Page_Program(uint8_t* dat, uint32_t WriteAddr, uint16_t nbytes)
{
    uint8_t cmd = PAGE_PROGRAM_CMD;
    
    WriteAddr <<= 8;
    
    W25QXX_Write_Enable();
    
    /* 使能片选 */
    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_RESET);
    
    SPI_Transmit(&cmd, 1);

    SPI_Transmit((uint8_t*)&WriteAddr, 3);
    
    SPI_Transmit(dat, nbytes);
    
    HAL_GPIO_WritePin(W25Q64_CHIP_SELECT_PORT, W25Q64_CHIP_SELECT_PIN, GPIO_PIN_SET);
    
    W25QXX_Wait_Busy();
}
