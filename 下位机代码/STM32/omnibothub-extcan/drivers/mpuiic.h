#ifndef _I2C_SOFT_H
#define	_I2C_SOFT_H

#include "stm32f4xx.h"
#include "time.h"

#define SCL_H_SPL         ANO_GPIO_I2C_SPL->BSRRL = I2C_Pin_SCL_SPL
#define SCL_L_SPL         ANO_GPIO_I2C_SPL->BSRRH = I2C_Pin_SCL_SPL
#define SDA_H_SPL         ANO_GPIO_I2C_SPL->BSRRL = I2C_Pin_SDA_SPL
#define SDA_L_SPL         ANO_GPIO_I2C_SPL->BSRRH = I2C_Pin_SDA_SPL
#define SCL_read_SPL      ANO_GPIO_I2C_SPL->IDR  & I2C_Pin_SCL_SPL
#define SDA_read_SPL      ANO_GPIO_I2C_SPL->IDR  & I2C_Pin_SDA_SPL

/***************I2C GPIO??******************/
#define ANO_GPIO_I2C_SPL	GPIOB
#define I2C_Pin_SCL_SPL		GPIO_Pin_1
#define I2C_Pin_SDA_SPL		GPIO_Pin_0
#define ANO_RCC_I2C_SPL		RCC_AHB1Periph_GPIOB
/*********************************************/
extern volatile u8 I2C_FastMode_SPL;

void I2c_Soft_Init_SPL(void);
void I2c_Soft_SendByte_SPL(u8 SendByte);
u8 I2c_Soft_ReadByte_SPL(u8);

//int I2c_Soft_Single_Write(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
//int I2c_Soft_Single_Read(u8 SlaveAddress,u8 REG_Address);
//int I2c_Soft_Mult_Read(u8 SlaveAddress,u8 REG_Address,u8 * ptChar,u8 size);

u8 IIC_Write_1Byte_SPL(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_1Byte_SPL(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);
u8 IIC_Write_nByte_SPL(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte_SPL(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);

#endif
