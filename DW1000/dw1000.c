#include "dw1000.h"

void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable GPIO used for DW1000 reset
	// done by main.c(MX_GPIO_Init) 

	//drive the RSTn pin low
	HAL_GPIO_WritePin(DW1000_RST_PORT, DW1000_RST_PIN, GPIO_PIN_RESET);
	
	//put the pin back to tri-state ... as input
	GPIO_InitStruct.Pin = DW1000_RST_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DW1000_RST_PORT, &GPIO_InitStruct);

    HAL_Delay(2);
}

static void SPI_ChangeRate(SPI_TypeDef *SPIx, uint16_t scalingfactor)
{
	uint16_t tmpreg = 0;

	/* Get the SPIx CR1 value */
	tmpreg = SPIx->CR1;

	/*clear the scaling bits*/
	tmpreg &= 0xFFC7;

	/*set the scaling bits*/
	tmpreg |= scalingfactor;

	/* Write to SPIx CR1 */
	SPIx->CR1 = tmpreg;
}

void spi_set_rate_low(void)
{
    SPI_ChangeRate(DW1000_SPI_Handle.Instance, SPI_BAUDRATEPRESCALER_32);
}

void spi_set_rate_high(void)
{
    SPI_ChangeRate(DW1000_SPI_Handle.Instance, SPI_BAUDRATEPRESCALER_4);
}
