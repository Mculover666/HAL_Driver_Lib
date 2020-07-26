#ifndef _W25Q64_H_
#define _W25Q64_H_

#include <spi.h>

#define W25Q64_CHIP_SELECT_PORT GPIOA
#define W25Q64_CHIP_SELECT_PIN  GPIO_PIN_4

#define SPI_Handle   hspi1

#define ManufactDeviceID_CMD    0x90
#define READ_STATU_REGISTER_1   0x05
#define READ_STATU_REGISTER_2   0x35
#define READ_DATA_CMD           0x03
#define WRITE_ENABLE_CMD        0x06
#define WRITE_DISABLE_CMD       0x04
#define SECTOR_ERASE_CMD        0x20
#define CHIP_ERASE_CMD          0xc7
#define PAGE_PROGRAM_CMD        0x02

/* W25Q64²Ù×÷º¯Êý */
uint16_t W25QXX_ReadID(void);
int W25QXX_Read(uint8_t* buffer, uint32_t start_addr, uint16_t nbytes);
void W25QXX_Write_Enable(void);
void W25QXX_Write_Disable(void);
void W25QXX_Erase_Sector(uint32_t sector_addr);
void W25QXX_Page_Program(uint8_t* dat, uint32_t WriteAddr, uint16_t byte_to_write);



#endif /* _BSP_W25Q64_H_ */
