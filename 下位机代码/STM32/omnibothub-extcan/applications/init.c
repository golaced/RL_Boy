#include "include.h" 
#include "flash.h"
#include "led_fc.h"
#include "usart_fc.h"
#include "spi.h"
#include "bat.h"
#include "beep.h"
#include "dog.h"
#include "pwm_in.h"
#include "pwm_out.h"
#include "stm32f4xx_dma.h"
#include "pressure.h"
#include "icm20602.h"
#include "ms5611.h"
#include "gps.h"
#include "outter_hml.h"
#include "mavl.h"
#include "can.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usb_conf.h" 
#include "usbd_cdc_core.h"
#include "usbd_cdc_vcp.h" 
#include "spl06.h"
#include "scheduler.h"
#include "locomotion_header.h"
#include "Custom_SPI_Device.h"
#include "wsled.h"
#include "i2c_soft.h"
#include "oled.h"
#include "test.h"

USB_OTG_CORE_HANDLE USB_OTG_dev;
u8 All_Init()
{
		char i;
		NVIC_PriorityGroupConfig(NVIC_GROUP);//设置系统中断优先级分组2
		SysTick_Configuration(); 	//
		//Delay_ms(100);		
	 	USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);
	  //Delay_ms(100);
	  POWER_INIT();
		RNG_Init();
		PWM_AUX_Out_Init(50); 	 
		PWM_Out_Init(50); 
		LED_Init();						   		//LED功能初始化
//------------------------Uart Init-------------------------------------
 
		Usart1_Init(9600);//轮毂电机 微雪*2 遥控器
		Usart2_Init(1000000);	//DJ总线
	  //Usart2_Init(250000);	//DJ总线
		serial_init();
		#if EN_DMA_UART1 
		MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF_SIZE1,1);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
		#endif
		#if EN_DMA_UART6 
		MYDMA_Config(DMA2_Stream6,DMA_Channel_5,(u32)&USART6->DR,(u32)SendBuff6,SEND_BUF_SIZE6,1);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
		#endif
		Usart3_Init(512000);			//extcan
		#if EN_D1MA_UART3
		MYDMA_Config(DMA1_Stream3,DMA_Channel_4,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF_SIZE3+2,0);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
		#endif
    Uart5_Init(100000);	    	//sbus
//	//-----------------DMA Init--------------------------
	#if EN_DMA_UART3 
		USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);       
		MYDMA_Enable(DMA1_Stream4,SEND_BUF_SIZE3+2);    
	#endif
	#if EN_DMA_UART1 
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);     
		MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE1+2);     
	#endif	
		#if EN_DMA_UART6
		USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);     
		MYDMA_Enable(DMA2_Stream6,SEND_BUF_SIZE6+2);     
	#endif	

  Delay_ms(100);		
  SPI3_Init();//IMU
	W25QXX_Init();Delay_ms(100);		
	READ_PARM();Delay_ms(100);

	#if defined(EN_BEEP)
	if(spi_master_connect_pi)
		Beep_Init(0,84-1);
	#endif
	
	#if defined(BOARD_FOR_CAN)&&0
		IWDG_Init(4,250);//100ms
	#endif
	
	Adc_Init();
	
	LED_Init_SCL_SDA();

	//--------------------CAN----------------------------1mbps 直接驱动MIT电机
	CAN1_Mode_Init(CAN_SJW_2tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	CAN2_Mode_Init(CAN_SJW_2tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	
	CAN_motor_init();

 	return (1);
}
