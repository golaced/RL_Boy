/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * ??   :????
 * ???  :i2c_soft.c
 * ??    :????i2c??
 * ??    :www.anotc.com
 * ??    :anotc.taobao.com
 * ??Q? :190169595
**********************************************************************************/
#include "mpuiic.h"

volatile u8 I2C_FastMode_SPL;

void I2c_Soft_delay_SPL()
{ 
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	
	if(!I2C_FastMode_SPL)
	{
		u8 i = 15;
		while(i--);
	}
}

void I2c_Soft_Init_SPL()
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  RCC_AHB1PeriphClockCmd(ANO_RCC_I2C_SPL , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL_SPL | I2C_Pin_SDA_SPL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ANO_GPIO_I2C_SPL, &GPIO_InitStructure);		
}

int I2c_Soft_Start_SPL()
{
	SDA_H_SPL;
	SCL_H_SPL;
	I2c_Soft_delay_SPL();
	if(!SDA_read_SPL)return 0;	//SDA?????????,??
	SDA_L_SPL;
	I2c_Soft_delay_SPL();
	if(SDA_read_SPL) return 0;	//SDA??????????,??
	SDA_L_SPL;
	I2c_Soft_delay_SPL();
	return 1;	

}

void I2c_Soft_Stop_SPL()
{
	SCL_L_SPL;
	I2c_Soft_delay_SPL();
	SDA_L_SPL;
	I2c_Soft_delay_SPL();
	SCL_H_SPL;
	I2c_Soft_delay_SPL();
	SDA_H_SPL;
	I2c_Soft_delay_SPL();
}

void I2c_Soft_Ask_SPL()
{
	SCL_L_SPL;
	I2c_Soft_delay_SPL();
	SDA_L_SPL;
	I2c_Soft_delay_SPL();
	SCL_H_SPL;
	I2c_Soft_delay_SPL();
	SCL_L_SPL;
	I2c_Soft_delay_SPL();
}

void I2c_Soft_NoAsk_SPL()
{
	SCL_L_SPL;
	I2c_Soft_delay_SPL();
	SDA_H_SPL;
	I2c_Soft_delay_SPL();
	SCL_H_SPL;
	I2c_Soft_delay_SPL();
	SCL_L_SPL;
	I2c_Soft_delay_SPL();
}

int I2c_Soft_WaitAsk_SPL(void) 	 //???:=1?ASK,=0?ASK
{
  u8 ErrTime = 0;
	SCL_L_SPL;
	I2c_Soft_delay_SPL();
	SDA_H_SPL;			
	I2c_Soft_delay_SPL();
	SCL_H_SPL;
	I2c_Soft_delay_SPL();
	while(SDA_read_SPL)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2c_Soft_Stop_SPL();
			return 1;
		}
	}
	SCL_L_SPL;
	I2c_Soft_delay_SPL();
	return 0;
}

void I2c_Soft_SendByte_SPL(u8 SendByte) //????????//
{
    u8 i=8;
    while(i--)
    {
        SCL_L_SPL;
        I2c_Soft_delay_SPL();
      if(SendByte&0x80)
        SDA_H_SPL;  
      else 
        SDA_L_SPL;   
        SendByte<<=1;
        I2c_Soft_delay_SPL();
				SCL_H_SPL;
				I2c_Soft_delay_SPL();
    }
    SCL_L_SPL;
}  

//?1???,ack=1?,??ACK,ack=0,??NACK
u8 I2c_Soft_ReadByte_SPL(u8 ask)  //????????//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H_SPL;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L_SPL;
      I2c_Soft_delay_SPL();
			SCL_H_SPL;
      I2c_Soft_delay_SPL();	
      if(SDA_read_SPL)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L_SPL;

	if (ask)
		I2c_Soft_Ask_SPL();
	else
		I2c_Soft_NoAsk_SPL();  
    return ReceiveByte;
} 


// IIC???????
u8 IIC_Write_1Byte_SPL(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
	I2c_Soft_Start_SPL();
	I2c_Soft_SendByte_SPL(SlaveAddress<<1);   
	if(I2c_Soft_WaitAsk_SPL())
	{
		I2c_Soft_Stop_SPL();
		return 1;
	}
	I2c_Soft_SendByte_SPL(REG_Address);       
	I2c_Soft_WaitAsk_SPL();	
	I2c_Soft_SendByte_SPL(REG_data);
	I2c_Soft_WaitAsk_SPL();   
	I2c_Soft_Stop_SPL(); 
	return 0;
}

// IIC?1????
u8 IIC_Read_1Byte_SPL(u8 SlaveAddress,u8 REG_Address,u8 *REG_data)
{      		
	I2c_Soft_Start_SPL();
	I2c_Soft_SendByte_SPL(SlaveAddress<<1); 
	if(I2c_Soft_WaitAsk_SPL())
	{
		I2c_Soft_Stop_SPL();
		return 1;
	}
	I2c_Soft_SendByte_SPL(REG_Address);     
	I2c_Soft_WaitAsk_SPL();
	I2c_Soft_Start_SPL();
	I2c_Soft_SendByte_SPL(SlaveAddress<<1 | 0x01);
	I2c_Soft_WaitAsk_SPL();
	*REG_data= I2c_Soft_ReadByte_SPL(0);
	I2c_Soft_Stop_SPL();
	return 0;
}	

// IIC?n????
u8 IIC_Write_nByte_SPL(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2c_Soft_Start_SPL();
	I2c_Soft_SendByte_SPL(SlaveAddress<<1); 
	if(I2c_Soft_WaitAsk_SPL())
	{
		I2c_Soft_Stop_SPL();
		return 1;
	}
	I2c_Soft_SendByte_SPL(REG_Address); 
	I2c_Soft_WaitAsk_SPL();
	while(len--) 
	{
		I2c_Soft_SendByte_SPL(*buf++); 
		I2c_Soft_WaitAsk_SPL();
	}
	I2c_Soft_Stop_SPL();
	return 0;
}

// IIC?n????
u8 IIC_Read_nByte_SPL(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf)
{	
	I2c_Soft_Start_SPL();
	I2c_Soft_SendByte_SPL(SlaveAddress<<1); 
	if(I2c_Soft_WaitAsk_SPL())
	{
		I2c_Soft_Stop_SPL();
		return 1;
	}
	I2c_Soft_SendByte_SPL(REG_Address); 
	I2c_Soft_WaitAsk_SPL();
	
	I2c_Soft_Start_SPL();
	I2c_Soft_SendByte_SPL(SlaveAddress<<1 | 0x01); 
	I2c_Soft_WaitAsk_SPL();
	while(len) 
	{
		if(len == 1)
		{
			*buf = I2c_Soft_ReadByte_SPL(0);
		}
		else
		{
			*buf = I2c_Soft_ReadByte_SPL(1);
		}
		buf++;
		len--;
	}
	I2c_Soft_Stop_SPL();
	return 0;
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

