/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include <string.h>

#include "deca_spi.h"
#include "deca_device_api.h"
#include "dw1000.h"

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
	// done by main.c, default SPI used is SPI1
	
	return 0;

} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{

	return 0;

} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
		HAL_StatusTypeDef status;
    decaIrqStatus_t  stat;

    stat = decamutexon() ;

		/* Enable DW1000_CS */
		HAL_GPIO_WritePin(DW1000_CS_PORT, DW1000_CS_PIN, GPIO_PIN_RESET);

		/* Send Header */
		status = HAL_SPI_Transmit(&DW1000_SPI_Handle, (uint8_t *)headerBuffer, headerLength, 0xFFFF);
		if (status != HAL_OK) {
			return -1;
		}
	
		/* Send Body */
		status = HAL_SPI_Transmit(&DW1000_SPI_Handle, (uint8_t *)bodyBuffer, bodylength, 0xFFFF);
		if (status != HAL_OK) {
			return -1;
		}
	
		/* Disable DW1000_CS */
    HAL_GPIO_WritePin(DW1000_CS_PORT, DW1000_CS_PIN, GPIO_PIN_SET);

    decamutexoff(stat) ;

    return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
		HAL_StatusTypeDef status;
    decaIrqStatus_t  stat;

    stat = decamutexon() ;

		/* Enable DW1000_CS */
		HAL_GPIO_WritePin(DW1000_CS_PORT, DW1000_CS_PIN, GPIO_PIN_RESET);

		/* Send Header */
		status = HAL_SPI_Transmit(&DW1000_SPI_Handle, (uint8_t *)headerBuffer, headerLength, 0xFFFF);
		if (status != HAL_OK) {
			return -1;
		}
	
		/* Receive Body */
		status = HAL_SPI_Receive(&DW1000_SPI_Handle, (uint8_t *)readBuffer, readlength, 0xFFFF);
		if (status != HAL_OK) {
			return -1;
		}
	
		/* Disable DW1000_CS */
    HAL_GPIO_WritePin(DW1000_CS_PORT, DW1000_CS_PIN, GPIO_PIN_SET);

    decamutexoff(stat) ;

    return 0;
} // end readfromspi()
