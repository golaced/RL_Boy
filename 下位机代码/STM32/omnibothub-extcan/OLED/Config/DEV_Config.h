/******************************************************************************
**************************Hardware interface layer*****************************
* | file      		:	DEV_Config.c
* |	version			:	V1.0
* | date			:	2020-06-17
* | function		:	Provide the hardware underlying interface	
******************************************************************************/
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

//#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_gpio.h"
#include "include.h"
#include <stdint.h>
#include <stdlib.h>

/**
 * data
**/
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

#define USE_SPI_4W 		0
#define USE_IIC 			1
#define USE_IIC_SOFT	1

#define I2C_ADR		0X3C //Hardware setting 0X3C or 0X3D

#define IIC_CMD		0X00
#define IIC_RAM		0X40

//OLED GPIO
#define OLED_CS_0		OLED_CS_GPIO_Port->BSRRL = OLED_CS_Pin//HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET)
#define OLED_CS_1		OLED_CS_GPIO_Port->BSRRH = OLED_CS_Pin//HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET)

#define OLED_DC_0		OLED_DC_GPIO_Port->BSRRL = OLED_DC_Pin//HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET)
#define OLED_DC_1		OLED_DC_GPIO_Port->BSRRH = OLED_DC_Pin//HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET)

#define OLED_RST_0		OLED_RST_GPIO_Port->BSRRL = OLED_RST_Pin//HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET)
#define OLED_RST_1		OLED_RST_GPIO_Port->BSRRH = OLED_RST_Pin//HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET)

//SPI GPIO
#define SPI1_SCK_0		SPI1_SCK_GPIO_Port->BSRRL = SPI1_SCK_Pin//HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_RESET)
#define SPI1_SCK_1		SPI1_SCK_GPIO_Port->BSRRH = SPI1_SCK_Pin//HAL_GPIO_WritePin(SPI1_SCK_GPIO_Port, SPI1_SCK_Pin, GPIO_PIN_SET)

#define SPI1_MOSI_0		OLED_DC_GPIO_Port->BSRRL = SPI1_MOSI_Pin//HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_RESET)
#define SPI1_MOSI_1		OLED_DC_GPIO_Port->BSRRH = SPI1_MOSI_Pin//HAL_GPIO_WritePin(SPI1_MOSI_GPIO_Port, SPI1_MOSI_Pin, GPIO_PIN_SET)
/*------------------------------------------------------------------------------------------------------*/

UBYTE 	System_Init(void);
void    System_Exit(void);

UBYTE 	SPI4W_Write_Byte(UBYTE value);
void 	I2C_Write_Byte(UBYTE value, UBYTE Cmd);

void Driver_Delay_ms(uint32_t xms);
void Driver_Delay_us(uint32_t xus);

#endif
