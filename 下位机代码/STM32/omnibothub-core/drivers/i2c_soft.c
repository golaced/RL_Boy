#include "i2c_soft.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//IIC 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

void GPIO_Set(GPIO_TypeDef* GPIOx,u32 BITx,u32 MODE,u32 OTYPE,u32 OSPEED,u32 PUPD)
{  
	u32 pinpos=0,pos=0,curpin=0;
	for(pinpos=0;pinpos<16;pinpos++)
	{
		pos=1<<pinpos;	//一个个位检查 
		curpin=BITx&pos;//检查引脚是否要设置
		if(curpin==pos)	//需要设置
		{
			GPIOx->MODER&=~(3<<(pinpos*2));	//先清除原来的设置
			GPIOx->MODER|=MODE<<(pinpos*2);	//设置新的模式 
			if((MODE==0X01)||(MODE==0X02))	//如果是输出模式/复用功能模式
			{  
				GPIOx->OSPEEDR&=~(3<<(pinpos*2));	//清除原来的设置
				GPIOx->OSPEEDR|=(OSPEED<<(pinpos*2));//设置新的速度值  
				GPIOx->OTYPER&=~(1<<pinpos) ;		//清除原来的设置
				GPIOx->OTYPER|=OTYPE<<pinpos;		//设置新的输出模式
			}  
			GPIOx->PUPDR&=~(3<<(pinpos*2));	//先清除原来的设置
			GPIOx->PUPDR|=PUPD<<(pinpos*2);	//设置新的上下拉
		}
	}
} 

//初始化IIC
void IIC_Init(void)
{					     
	RCC->AHB1ENR|=1<<0;    //使能PORTA时钟	   	  
	GPIO_Set(GPIOA,PIN2|PIN3,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PB8/PB9设置 
	IIC_SCL=1;
	IIC_SDA=1;
}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

 void OLED_write_cmd(uint8_t cmd)
{
	IIC_Start();
	IIC_Send_Byte(0x78);   //从机地址，SA0=0,RW=0 
	IIC_Wait_Ack();
	
	IIC_Send_Byte(0x00);   //控制字节，Co=0,D/C#=0
	IIC_Wait_Ack();
	
	IIC_Send_Byte(cmd);   //命令码
	IIC_Wait_Ack();
	
	IIC_Stop();
}



 void OLED_write_data(uint8_t dat)
{
	IIC_Start();
	
	IIC_Send_Byte(0x78);   // 从机地址，SA0=0,RW=0
	IIC_Wait_Ack();
	
	IIC_Send_Byte(0x40);   //控制字节，Co=0,D/C#=1
	IIC_Wait_Ack();
	
	IIC_Send_Byte(dat);   //数据值
	IIC_Wait_Ack();
	
	IIC_Stop();
}


static void OLED_write_data_array(const uint8_t* dats,uint32_t len)
{
	IIC_Start();
	IIC_Send_Byte(0x78);   //从机地址，SA0=0,RW=0 
	IIC_Wait_Ack();
	
	IIC_Send_Byte(0x40);   //控制字节，Co=0,D/C#=1
	IIC_Wait_Ack();
	
	for(uint32_t i=0;i<len;i++)
	{
		IIC_Send_Byte(dats[i]);   //数据值
		IIC_Wait_Ack();
	}
	IIC_Stop();
}

void OLED_Clear()
{
    UBYTE Column,Page;
    for(Page = 0; Page < OLED_0in91_HEIGHT/8; Page++) {
        OLED_write_cmd(0xb0 + Page);    //Set page address
        OLED_write_cmd(0x00);           //Set display position - column low address
        OLED_write_cmd(0x10);           //Set display position - column high address
        for(Column = 0; Column < OLED_0in91_WIDTH; Column++)
            OLED_write_cmd(0x00);
    }
//		unsigned char i,j;
//	
//	OLED_write_cmd(0X00);		// 水平寻址模式 
//	OLED_write_cmd(0X21);		// 设置列起始和结束地址
//	OLED_write_cmd(0X00);		// 列起始地址 0
//	OLED_write_cmd(0X7F);		// 列终止地址 127
//	OLED_write_cmd(0X22);		// 设置页起始和结束地址
//	OLED_write_cmd(0X00);		// 页起始地址 0
//	OLED_write_cmd(0X07);		// 页终止地址 7
//	
//	for(i=0; i<8; i++)		// 写入一帧'0'
//		for(j=0; j<128; j++)
//			OLED_write_cmd(0X00);
//	
//	IIC_Stop();
}


//0.91寸IIC接口OLED初始化
void OLED_init(void)
{
		IIC_Init();     //初始化驱动I2C总线的引脚
	//延时200ms等待OLED稳定
		Delay_ms(200);
		OLED_write_cmd(0xAE);
    OLED_write_cmd(0x40);//---set low column address
    OLED_write_cmd(0xB0);//---set high column address
    OLED_write_cmd(0xC8);//-not offset
    OLED_write_cmd(0x81);
    OLED_write_cmd(0xff);
    OLED_write_cmd(0xa1);
    OLED_write_cmd(0xa6);
    OLED_write_cmd(0xa8);
    OLED_write_cmd(0x1f);
    OLED_write_cmd(0xd3);
    OLED_write_cmd(0x00);
    OLED_write_cmd(0xd5);
    OLED_write_cmd(0xf0);
    OLED_write_cmd(0xd9);
    OLED_write_cmd(0x22);
    OLED_write_cmd(0xda);
    OLED_write_cmd(0x02);
    OLED_write_cmd(0xdb);
    OLED_write_cmd(0x49);
    OLED_write_cmd(0x8d);
    OLED_write_cmd(0x14);
		OLED_write_cmd(0xaf); //打开显示
		Delay_ms(500);
    OLED_Clear();     //清除显示
}


















