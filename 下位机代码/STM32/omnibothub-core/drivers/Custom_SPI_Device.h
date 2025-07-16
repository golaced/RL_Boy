/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CUSTOM_SPI_DEVICE_H
#define __CUSTOM_SPI_DEVICE_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdio.h>
#include "string.h"

#define USE_WHEEL     0
/* Define config -------------------------------------------------------------*/
#define CAN_LINK_COMM_VER1 1//new version for param divded 新协议  电机数量需要在节点盒另外配置

#define SEND_DIV_SPI 0
#define SPI_IS_BIG  1
#define	SLAVE_SPI_BAUDRATE					(SPI_BaudRatePrescaler_8)	//无用  SPI由Linux作为主机设置  
#define DataSize								162//SPI数据量 需要与Linux一致 且为二者发送最大量的一个
#define CheckSumSize						(0)


#define SPI_DEVICE							SPI2
#define SPI_DEVICE_CLK						RCC_APB1Periph_SPI2
#define SPI_DEVICE_CLK_INIT             		RCC_APB1PeriphClockCmd 

#define SPI_DEVICE_IRQn						SPI2_IRQn

#define SPI_DEVICE_SCK_PIN					GPIO_Pin_13
#define SPI_DEVICE_SCK_GPIO_PORT			GPIOB
#define SPI_DEVICE_SCK_GPIO_SOURCE			GPIO_PinSource13
#define SPI_DEVICE_SCK_GPIO_AF				GPIO_AF_SPI2
#define SPI_DEVICE_SCK_GPIO_CLK				RCC_AHB1Periph_GPIOB

#define SPI_DEVICE_MISO_PIN 				GPIO_Pin_14
#define SPI_DEVICE_MISO_GPIO_PORT			GPIOB
#define SPI_DEVICE_MISO_GPIO_SOURCE		GPIO_PinSource14
#define SPI_DEVICE_MISO_GPIO_AF			GPIO_AF_SPI2
#define SPI_DEVICE_MISO_GPIO_CLK			RCC_AHB1Periph_GPIOB

#define SPI_DEVICE_MOSI_PIN					GPIO_Pin_15
#define SPI_DEVICE_MOSI_GPIO_PORT			GPIOB
#define SPI_DEVICE_MOSI_GPIO_SOURCE		GPIO_PinSource15
#define SPI_DEVICE_MOSI_GPIO_AF			GPIO_AF_SPI2
#define SPI_DEVICE_MOSI_GPIO_CLK			RCC_AHB1Periph_GPIOB

//GPIO
#define SPI_DEVICE_CS_PIN					GPIO_Pin_12
#define SPI_DEVICE_CS_GPIO_PORT			GPIOB
#define SPI_DEVICE_CS_GPIO_SOURCE			GPIO_PinSource12
//#define SPI_DEVICE_CS_GPIO_AF				GPIO_AF_SPI2
//#define SPI_DEVICE_CS_GPIO_CLK				RCC_AHB1Periph_GPIOB

//slave only
#define SPI_DEVICE_CS_EXTI_PortSource		EXTI_PortSourceGPIOB
#define SPI_DEVICE_CS_EXTI_PinSource			EXTI_PinSource12
#define SPI_DEVICE_CS_EXTI_Line				EXTI_Line12
#define SPI_DEVICE_CS_EXTI_IRQn				EXTI15_10_IRQn

//#define SPI_NCS_LOW(void)       			(GPIO_ResetBits(SPI_DEVICE_CS_GPIO_PORT, SPI_DEVICE_CS_PIN))
//#define SPI_NCS_HIGH(void)      			(GPIO_SetBits(SPI_DEVICE_CS_GPIO_PORT, SPI_DEVICE_CS_PIN))  
#define Is_SPI_NCS_LOW(void)       			(GPIO_ReadInputDataBit(SPI_DEVICE_CS_GPIO_PORT, SPI_DEVICE_CS_PIN))

/* Macro ---------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
void Custom_SPI_DMABufferStart(void);
void Custom_SPI_DMABufferWait(void);
void Custom_SPI_DEVICE_TestCommand(void);
void Custom_SPI_DMABufferConfig(void);

void Custom_SPI_DEVICE_Slave_Config(void);
void Custom_SPI_DEVICE_Slave_EXTI_Config(void);

uint8_t Get_CheckSum(uint8_t data[],uint32_t len);

/* Exported constants --------------------------------------------------------*/

#endif  /* __CUSTOM_SPI_DEVICE_H */

