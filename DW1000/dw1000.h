#ifndef _DW1000_H_
#define _DW1000_H_

#include "stm32l4xx_hal.h"

/*
	CS <-------------  PA4
	RST <------------- PB9
	WAKEUP <---------- PA8
*/
#define DW1000_CS_PORT			GPIOA
#define DW1000_CS_PIN			GPIO_PIN_4
#define DW1000_RST_PORT			GPIOB
#define DW1000_RST_PIN			GPIO_PIN_9
#define DW1000_WAKEUP_PORT	    GPIOA
#define DW10000_WAKEUP_PIN	    GPIO_PIN_8

/*
	IRQ --------------> PC9
*/
#define DW1000_IRQn_TYPE		EXTI9_5_IRQn
#define DW1000_IRQ_PORT		    GPIOC
#define DW1000_IRQ_PIN			GPIO_PIN_9

/*
	SPI Interface <---> SPI1
	SPI_CS <----------> PA4
	SPI_CLK <---------> PA1
	SPI_MISO <--------> PA6
	SPI_MOSI <--------> PA12
*/
extern SPI_HandleTypeDef hspi1;
#define DW1000_SPI_Handle hspi1

void reset_DW1000(void);
void spi_set_rate_low(void);
void spi_set_rate_high(void);

#endif /* _DW1000_H_ */
