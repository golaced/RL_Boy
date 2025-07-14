#include "include.h"
#include "usart_fc.h"
#include "sbus.h"
#include "gait_math.h"
#include "gps.h"
#include "fifo.h"
#include "mavl.h"
#include "nav.h"
#include "LobotSerialServo.h"
#include "usbd_cdc_vcp.h"
#include "spi.h"
#include "led_fc.h"
#include "wsled.h"
#include <stdio.h> 	
#include "SCSCL.h"
#include "SCServo.h"
#include "SMS_STS.h"
#include "can.h"

_ODOMETER flow;
_PI pi;
MOUDLE module;
RC_GETDATA Rc_Get;
M100 m100,px4;
_ARMSS arm_cmd_s;
 _IMUO imuo;
 _WHEEL_WX _wheel_wx[4];
 _WHEEL_2Dof _wheel_2d;
 _Robot robot;
 
int id_test=0;
int pwm_dj_test=1500;
_Palm_DJ palm_dj;
 //UART 读数据缓冲区
__IO uint8_t uartBuf[128];
__IO int head = 0;
__IO int tail  = 0;
void Usart1_Init(u32 br_num)//-------Radio
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	#if USE_SERVO1
		#if USE_VR
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		#else
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
		#endif
	#else
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	#endif
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART1, ENABLE); 
}

void Usart2_Init(u32 br_num)//--GPS
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART2, ENABLE);  //使能串口1 
	USART_ClearFlag(USART2, USART_FLAG_TC);
	//使能USART2接收中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART2, ENABLE); 
}

void Usart3_Init(u32 br_num)//-------PI
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
   //USART3 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	//使能USART3接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART3
	USART_Cmd(USART3, ENABLE); 
//	//使能发送（进入移位）中断
}

void Usart4_Init(u32 br_num)//-------unused
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(UART4, ENABLE);  //使能串口1 
	USART_ClearFlag(UART4, USART_FLAG_TC);
	//使能USART2接收中断
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(UART4, ENABLE); 
}


void Uart5_Init(u32 br_num)//-----Sbus
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode =  USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART5, &USART_InitStructure);
	//使能UART5接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART5, ENABLE); 
}

void Uart6_Init(u32 br_num)//-------IDLE
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = br_num;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART6, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART6, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART6, USART_FLAG_TC);
	

	//使能USART2接收中断
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART6, ENABLE); 
}

void UsartSend1(uint8_t ch)
{
while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, ch); 
}

void Send_buf1(u8 *dataToSend , char length)
{
	int i;
  for(i=0;i<length;i++)
     UsartSend1(dataToSend[i]);
}

void UsartSend2(uint8_t ch)
{
while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, ch); 
}
void UsartSend3(uint8_t ch)
{
while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
USART_SendData(USART3, ch); 
}

void UsartSend6(uint8_t ch)
{
while(USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
USART_SendData(USART6, ch); 
}


//For engien
u8 Radio_Start[]={"#000P1501T0000!\r\n"};
#define SRadio_Start sizeof(Radio_Start)	
float Radio_Startt=6;//s

u8 Radio_Slow[]={"#000P1502T0000!\r\n"};
#define SRadio_Slow sizeof(Radio_Slow)	
float Radio_Slowt=54+60;//s

u8 Radio_Fast[]={"#000P1503T0000!\r\n"};
#define SRadio_Fast sizeof(Radio_Fast)	
float Radio_Fastt=60+50;//s

u8 Radio_Down[]={"#000P1504T0000!\r\n"};
#define SRadio_Down sizeof(Radio_Down)	
float Radio_Downt=5;//s

u8 Radio_Off[]={"#000PDST0!\r\n"};
#define SRadio_Off sizeof(Radio_Off)	
void UsartSend4(uint8_t ch)
{
while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
USART_SendData(UART4, ch); 
}

void radio_off(void){
char i=0;
	for(i=0;i<SRadio_Off;i++)
	{
	UsartSend3(Radio_Off[i]);
	}
}
	

void radio_control(char sel,float dt)
{
	char i=0;
	static char state=0;
	static char flag_tx=0,flag_finish=0;
	static float timer=0;
	static char ocu_reg,gait_mode_reg;
	char ocu_mode=ocu.cmd_robot_state;
	static float time_check;
	static int size;
	static u8 *Radio;
	
	switch(state){
		case 0:
			if(ocu_reg==0&&(ocu_mode==1||ocu_mode==12))//正确切换
			{
				flag_finish=0;timer=0;
				flag_tx=1;
				size=SRadio_Start;
				Radio=&Radio_Start[0];
				time_check=Radio_Startt;
				state=1;
			}
			else if(ocu_reg==0&&ocu_mode)//异常切换
			{
				flag_finish=0;timer=0;
				flag_tx=1;
				size=SRadio_Slow;
				Radio=&Radio_Slow[0];
				time_check=Radio_Slowt;
				state=2;
			}
		break;
		case 1:
			timer+=dt;
			if(timer>Radio_Startt)
			{
				flag_finish=1;
			}
			if(flag_finish){
				flag_finish=0;timer=0;
				state=2;
				flag_tx=1;
				size=SRadio_Slow;
				Radio=&Radio_Slow[0];
				time_check=Radio_Slowt;
			}
		break;	
		case 2://正常运行
			timer+=dt;
			if(gait_mode_reg!=F_TROT&&vmc_all.gait_mode==F_TROT)
			{
				flag_finish=0;timer=0;
				flag_tx=1;
				size=SRadio_Fast;
				Radio=&Radio_Fast[0];
				time_check=Radio_Fastt;
			}
			if(gait_mode_reg==TROT&&vmc_all.gait_mode!=TROT)
			{
				flag_finish=0;timer=0;
				flag_tx=1;
				size=SRadio_Slow;
				Radio=&Radio_Slow[0];
				time_check=Radio_Slowt;
			}	
			if(ocu_mode==0)
			{
				flag_finish=0;timer=0;
				flag_tx=1;
				size=SRadio_Down;
				Radio=&Radio_Down[0];
				time_check=Radio_Downt;
				state=3;
			}	
			
			if(timer>time_check/2&&time_check>0){//重复播放
				flag_finish=0;timer=0;
				flag_tx=1;
			}		
		break;
		case 3:
			timer+=dt;
			if(timer>time_check)
			{
				flag_finish=1;
			}
			if(flag_finish){
				flag_finish=0;timer=0;
				state=0;
			}
		break;
		
	}
	
	gait_mode_reg=vmc_all.gait_mode;
	ocu_reg=ocu.cmd_robot_state;
	if(flag_tx){
	flag_tx=0;
		for(i=0;i<size;i++)
		{
		UsartSend3(*(Radio+i));
		}
	}
}

//图像控制命令
void Data_LEG_CMD(u8 *data_buf,u8 num)
{ static u8 cnt[4];
	u8 id;
	vs16 rc_value_temp;
	u8 sum = 0,sum_rx=0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	sum_rx=*(data_buf+num-1);
	if(!(sum==*(data_buf+num-1))&&*(data_buf+2)!=MODE_FACE)		return;		//判断sum
	//if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==MODE_CUBE)//Cube
  { 
	  pi.connect=MODE_CUBE;
		pi.lost_cnt=0;
		pi.cube.check=*(data_buf+4);
	  pi.cube.x=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6))-320/2;
		pi.cube.y=-(((int16_t)(*(data_buf+7)<<8)|*(data_buf+8))-240/2);
		pi.cube.s=((int16_t)(*(data_buf+9)<<8)|*(data_buf+10));
		
		pi.cube.pos[Xr]=(float)((int16_t)(*(data_buf+11)<<8)|*(data_buf+12))/100.;
		pi.cube.pos[Yr]=(float)((int16_t)(*(data_buf+13)<<8)|*(data_buf+14))/100.;
		pi.cube.pos[Zr]=(float)((int16_t)(*(data_buf+15)<<8)|*(data_buf+16))/100.;
		
		pi.cube.att[Xr]=(float)((int16_t)(*(data_buf+17)<<8)|*(data_buf+18))/100.;
		pi.cube.att[Yr]=(float)((int16_t)(*(data_buf+19)<<8)|*(data_buf+20))/100.;
		pi.cube.att[Zr]=(float)((int16_t)(*(data_buf+21)<<8)|*(data_buf+22))/100.;
	}	
  else if(*(data_buf+2)==MODE_COLOR)//Color Ball
  { 
	  pi.connect=MODE_COLOR;
		pi.lost_cnt=0;
		pi.color.check=*(data_buf+4);
	  pi.color.x=((int16_t)(*(data_buf+5)<<8)|*(data_buf+6))-320/2;
		pi.color.y=-(((int16_t)(*(data_buf+7)<<8)|*(data_buf+8))-240/2);
		pi.color.s=((int16_t)(*(data_buf+9)<<8)|*(data_buf+10));
		
		pi.color.pos[Xr]=(float)((int16_t)(*(data_buf+11)<<8)|*(data_buf+12))/100.;
		pi.color.pos[Yr]=(float)((int16_t)(*(data_buf+13)<<8)|*(data_buf+14))/100.;
		pi.color.pos[Zr]=(float)((int16_t)(*(data_buf+15)<<8)|*(data_buf+16))/100.;
		
		pi.color.att[Xr]=(float)((int16_t)(*(data_buf+17)<<8)|*(data_buf+18))/100.;
		pi.color.att[Yr]=(float)((int16_t)(*(data_buf+19)<<8)|*(data_buf+20))/100.;
		pi.color.att[Zr]=(float)((int16_t)(*(data_buf+21)<<8)|*(data_buf+22))/100.;
	}	
	else if(*(data_buf+2)==MODE_LINE)//LINE
  { 
	  pi.connect=MODE_LINE;
		pi.lost_cnt=0;
		pi.line.check=*(data_buf+4);
	  pi.line.x=-(((int16_t)(*(data_buf+5)<<8)|*(data_buf+6))-250/2);
		pi.line.y=-(((int16_t)(*(data_buf+7)<<8)|*(data_buf+8))-120/2);
		pi.line.angle=(float)((int16_t)(*(data_buf+9)<<8)|*(data_buf+10))/100.;
	}	
	else if(*(data_buf+2)==MODE_FACE)//Face
  { 
	  pi.connect=MODE_FACE;
		pi.lost_cnt=0;
		pi.face.check=*(data_buf+4);
	  pi.face.x=*(data_buf+5)-256/2;
		pi.face.y=-(*(data_buf+6)-192/2);
		pi.face.s=*(data_buf+7);
	}	
	else if(*(data_buf+2)==MODE_CMD)//CMD
  { 
	  pi.connect=MODE_CMD;
		pi.lost_cnt=0;
	  pi.cmd_mode=*(data_buf+4);
		pi.cmd_spd[Xr]=(float)((int16_t)(*(data_buf+5)<<8)|*(data_buf+6))/100.;
		pi.cmd_spd[Yr]=(float)((int16_t)(*(data_buf+7)<<8)|*(data_buf+8))/100.;
		pi.cmd_spd[Zr]=(float)((int16_t)(*(data_buf+9)<<8)|*(data_buf+10))/100.*57.3;
		
		pi.cmd_att[PITr]=(float)((int16_t)(*(data_buf+11)<<8)|*(data_buf+12))/10.;
		pi.cmd_att[ROLr]=(float)((int16_t)(*(data_buf+13)<<8)|*(data_buf+14))/10.;
		pi.cmd_att[YAWr]=(float)((int16_t)(*(data_buf+15)<<8)|*(data_buf+16))/10.;
		
		pi.cmd_pos[Xr]=(float)((int16_t)(*(data_buf+17)<<8)|*(data_buf+18))/100.;
		pi.cmd_pos[Yr]=(float)((int16_t)(*(data_buf+19)<<8)|*(data_buf+20))/100.;
		pi.cmd_pos[Zr]=(float)((int16_t)(*(data_buf+21)<<8)|*(data_buf+22))/100.;
		
		pi.cmd_spd[Xr]=LIMIT(pi.cmd_spd[Xr],-MAX_SPD,MAX_SPD);
		pi.cmd_spd[Yr]=LIMIT(pi.cmd_spd[Yr],-MAX_SPD,MAX_SPD);
		pi.cmd_spd[Zr]=LIMIT(pi.cmd_spd[Zr],-MAX_SPD_RAD,MAX_SPD_RAD);
		pi.cmd_att[PITr]=LIMIT(pi.cmd_att[PITr],-25,25);
		pi.cmd_att[ROLr]=LIMIT(pi.cmd_att[ROLr],-25,25);
		pi.cmd_att[YAWr]=LIMIT(pi.cmd_att[YAWr],-180,180);
	}			
}

//数传控制命令
_LINK_CMD o_cmd;
void Anal_Outter_CMD(u8 *data_buf,u8 num)
{ static u8 cnt[4];
	u8 id;
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
  if(*(data_buf+2)==1)//
  { 
	  o_cmd.connect=1;
		o_cmd.lost_cnt=0;
		o_cmd.mode=*(data_buf+4);
	  o_cmd.spd[Xr]=(float)((int16_t)(*(data_buf+5)<<8)|*(data_buf+6))/100.;
		o_cmd.spd[Yr]=(float)((int16_t)(*(data_buf+7)<<8)|*(data_buf+8))/100.;
		o_cmd.spd[Zr]=(float)((int16_t)(*(data_buf+9)<<8)|*(data_buf+10))/100.;
		o_cmd.att[PITr]=(float)((int16_t)(*(data_buf+11)<<8)|*(data_buf+12))/10.;
		o_cmd.att[ROLr]=(float)((int16_t)(*(data_buf+13)<<8)|*(data_buf+14))/10.;
		o_cmd.att[YAWr]=(float)((int16_t)(*(data_buf+15)<<8)|*(data_buf+16))/10.;
		o_cmd.pos[Xr]=(float)((int16_t)(*(data_buf+17)<<8)|*(data_buf+18))/100.;
		o_cmd.pos[Yr]=(float)((int16_t)(*(data_buf+19)<<8)|*(data_buf+20))/100.;
		o_cmd.pos[Zr]=(float)((int16_t)(*(data_buf+21)<<8)|*(data_buf+22))/100.;
		
		o_cmd.spd[Xr]=LIMIT(o_cmd.spd[Xr],-MAX_SPD,MAX_SPD);
		o_cmd.spd[Yr]=LIMIT(o_cmd.spd[Yr],-MAX_SPD,MAX_SPD);
		o_cmd.spd[Zr]=LIMIT(o_cmd.spd[Zr],-MAX_SPD_RAD,MAX_SPD_RAD);
		o_cmd.att[PITr]=LIMIT(o_cmd.att[PITr],-25,25);
		o_cmd.att[ROLr]=LIMIT(o_cmd.att[ROLr],-25,25);
		o_cmd.att[YAWr]=LIMIT(o_cmd.att[YAWr],-180,180);
	}
}


u8 UART_RX_BUF[16];
char isUartRxCompleted = 0;
#if defined(USE_SERIAL_SERVO)
void USART1_IRQHandler(void)
{ OSIntEnter(); 
	uint8_t Res;
	static char isGotFrameHeader = 0;
	static uint8_t frameHeaderCount = 0;
	static uint8_t dataLength = 2;
	static uint8_t dataCount = 0;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) { //??????
		Res = USART_ReceiveData(USART1);//(USART1->DR);	//?????????
		if (!isGotFrameHeader) {  //????
			if (Res == 0x55) {
				frameHeaderCount++;
				if (frameHeaderCount == 2) {
					frameHeaderCount = 0;
					isGotFrameHeader = 1;
					dataCount = 1;
				}
			} else {
				isGotFrameHeader = 0;
				dataCount = 0;
				frameHeaderCount = 0;
			}
		}
		if (isGotFrameHeader) { //????????
			UART_RX_BUF[dataCount] = Res;
			if (dataCount == 3) {
				dataLength = UART_RX_BUF[dataCount];
				if (dataLength < 3 || dataLength > 7) {
					dataLength = 3;
					isGotFrameHeader = 0;
				}
			}
			dataCount++;
			if (dataCount == dataLength + 3) {
				if (isUartRxCompleted == false) {
					isUartRxCompleted = true;
					memcpy(LobotRxBuf, UART_RX_BUF, dataCount+2);	
					LobotSerialMsgHandle();
				}
				isGotFrameHeader = false;
			}
		}
		
	}
	OSIntExit(); 
}
#endif

char isRxCompleted(void)
{
	if(isUartRxCompleted == 1){
		isUartRxCompleted = 0;
		return 1;
	}else{
		return 0;
	}
}

u8 TxBuffer1[256];
u8 TxCounter1=0;
u8 count1=0; 
u8 RxBuffer1[50];
char RxBuffer1c[50];
u8 RxState1 = 0;
u8 RxBufferNum1 = 0;
int RxBufferCnt1 = 0;
u8 RxLen1 = 0;
static u8 _data_len1 = 0,_data_cnt1 = 0;
int _data_cnt11=0;
int RxBufferCnt11=0;
 
void USART1_IRQHandler(void)//Radio  mavlink
{ //OSIntEnter(); 
	u8 com_data,crc_byte=0,i=0;
	_OCU ocu_rx;
	uint8_t c,t;
	char crc_check=0;
	if(USART1->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART1->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志

		com_data = USART1->DR;
		#if USE_SERVO1
			uartBuf[tail] = com_data;
			tail = (tail+1)%128;
		#endif
		if(RxState1==0&&com_data==0xAA)
		{
			RxState1=1;
			RxBuffer1[0]=com_data;
		}
		else if(RxState1==1)
		{
			if(com_data==0xBB){
				RxBuffer1[1]=com_data;
				RxState1=2;
				_data_cnt1=0;
			}
			else
				RxState1=0;
		}
		else if(RxState1==2)
		{
			RxBuffer1[2+_data_cnt1++]=com_data;
			
			if(_data_cnt1>17-1)
			{	
				for(int i=0;i<18;i++)
					crc_check+=RxBuffer1[i];
				if(crc_check==RxBuffer1[18]){
					ocu.key_st=RxBuffer1[9]-1;
					ocu.key_back=RxBuffer1[8]-1;
					ocu.key_lr=-(RxBuffer1[3]-1);
					ocu.key_ud=RxBuffer1[2]-1;
					ocu.key_x=RxBuffer1[10]-1;
					ocu.key_a=RxBuffer1[12]-1;
					ocu.key_b=RxBuffer1[13]-1;
					ocu.key_y=RxBuffer1[11]-1;
					ocu.key_ll=RxBuffer1[4]-1;
					ocu.key_rr=RxBuffer1[5]-1;
					ocu.rc_spd_w[Xr]=my_deathzoom((float)(RxBuffer1[14]-125)/125.0,0.05);
					ocu.rc_spd_w[Yr]=my_deathzoom(-(float)(RxBuffer1[15]-125)/125.0,0.05);
					ocu.rc_att_w[PITr]=my_deathzoom((float)(RxBuffer1[16]-125)/125.0,0.05);
					ocu.rc_att_w[ROLr]=my_deathzoom(-(float)(RxBuffer1[17]-125)/125.0,0.05);
					ocu.rate_yaw_w=-(RxBuffer1[6]*-0.5+RxBuffer1[7]*0.5);
						
					ocu.connect=1;ocu.loss_cnt=0;
					ocu.mode=2;
				}				
				
			RxState1=_data_len1=0;
			}
		}
		else
			RxState1 = 0;
		
		if(!wsled.led_lock&&0){
			if(RxState1==0&&(com_data==0+WHEEL_ID_OFF||com_data==1+WHEEL_ID_OFF||com_data==2+WHEEL_ID_OFF||com_data==3+WHEEL_ID_OFF))// 电机ID设置为10 11
			{
				RxState1=1;
				RxBuffer1c[0]=com_data;
			}
			else if(RxState1==1&&com_data==0x02)//速度模式 
			{
				RxState1=2;
				RxBuffer1c[1]=com_data;
				_data_len1 = 8;
				_data_cnt11 = 0;
			}
			else if(RxState1==2)//doghomewx
			{
				RxBuffer1c[2+_data_cnt11++]=com_data;
				RxBufferCnt11++;
				if(_data_cnt11>7){
					RxState1=0;
					RxBufferCnt1++;
					crc_check=PY_CRC_8_T(RxBuffer1c,9);
					if(crc_check==RxBuffer1c[9]){//检测正确
						int id=LIMIT(RxBuffer1c[0]-WHEEL_ID_OFF,0,99);
						_wheel_wx[id].connect=1;
						_wheel_wx[id].dt_rx=Get_Cycle_T(30+(id)); 
						_wheel_wx[id].rx_cnt++;
						_wheel_wx[id].loss_cnt=0;
						_wheel_wx[id].mode=RxBuffer1c[1];
						_wheel_wx[id].current=(float)((int16_t)(*(RxBuffer1c+2)<<8)|*(RxBuffer1c+3))/32767.*8.0*_wheel_wx[id].cmd_flag;
						_wheel_wx[id].rad_now=(float)((int16_t)(*(RxBuffer1c+4)<<8)|*(RxBuffer1c+5))*(2.0 * M_PI / 60.0)*_wheel_wx[id].cmd_flag;
						_wheel_wx[id].q_now=(float)((u16)(*(RxBuffer1c+6)<<8)|*(RxBuffer1c+7))/32767.*360*_wheel_wx[id].cmd_flag;
						_wheel_wx[id].tau_now=_wheel_wx[id].current*_wheel_wx[id].i_2_tau;
						_wheel_wx[id].v_now=_wheel_wx[id].rad_now*_wheel_wx[id].r;
						_wheel_wx[id].err=RxBuffer1c[8];
					}
				}
			}else
				RxState1=0;
		}
	 
	}
//轮毂电机微雪 
	 
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {   		
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  }		
}


u8 TxBuffer2[256];
u8 TxCounter2=0;
u8 count2=0; 
u8 RxBuffer2[80];
u8 RxState2 = 0;
u8 RxBufferNum2 = 0;
u8 RxBufferCnt2 = 0;
u8 RxLen2 = 0;
static u8 _data_len2 = 0,_data_cnt2 = 0;
void USART2_IRQHandler(void)//GPS
{ //OSIntEnter(); 
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART2->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
		GPS_IRQ(com_data);
		#if !USE_SERVO1
		uartBuf[tail] = com_data;
		tail = (tail+1)%128;
		#endif
		if(RxState2==0&&com_data==0xAA)
		{
			RxState2=1;
			RxBuffer2[0]=com_data;
		}
		else if(RxState2==1&&com_data==0xAF)
		{
			RxState2=2;
			RxBuffer2[1]=com_data;
		}
		else if(RxState2==2&&com_data>0&&com_data<0XF1)
		{
			RxState2=3;
			RxBuffer2[2]=com_data;
		}
		else if(RxState2==3&&com_data<80)
		{
			RxState2 = 4;
			RxBuffer2[3]=com_data;
			_data_len2 = com_data;
			_data_cnt2 = 0;
		}
		else if(RxState2==4&&_data_len2>0)
		{
			_data_len2--;
			RxBuffer2[4+_data_cnt2++]=com_data;
			if(_data_len2==0)
				RxState2= 5;
		}
		else if(RxState2==5)
		{
			RxState2 = 0;
			RxBuffer2[4+_data_cnt2]=com_data;
   
		}
		else
			RxState2 = 0;
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(USART2,USART_IT_TXE ) )
	{
				
		USART2->DR = TxBuffer2[TxCounter2++]; //写DR清除中断标志          
		if(TxCounter2 == count2)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
}



//图像控制命令
void Data_ExtCAN_FB(u8 *data_buf,u8 num)
{ static u8 cnt[4];
	u8 id;
	vs16 rc_value_temp;
	u8 sum = 0,sum_rx=0;
	u8 i;
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	sum_rx=*(data_buf+num-1);
	if(!(sum==*(data_buf+num-1))&&*(data_buf+2)!=MODE_FACE)		return;		//判断sum
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xBA && *(data_buf+1)==0xBF))		return;		//判断帧头
  if(*(data_buf+2)==0x01)//FB1
  { 
		palm_dj.connect_link=1;
		palm_dj.loss_cnt=0;
		palm_dj.connect[0]=*(data_buf+4);
		palm_dj.connect[1]=*(data_buf+5);
		palm_dj.connect[2]=*(data_buf+6);
		palm_dj.connect[3]=*(data_buf+7);
		palm_dj.connect[4]=*(data_buf+8);
		palm_dj.connect[5]=*(data_buf+9);
		palm_dj.connect[6]=*(data_buf+10);
		palm_dj.connect[7]=*(data_buf+11);
		palm_dj.connect[8]=*(data_buf+12);
		palm_dj.connect[9]=*(data_buf+13);
		palm_dj.connect[10]=*(data_buf+14);
		palm_dj.connect[11]=*(data_buf+15);
		palm_dj.connect[12]=*(data_buf+16);
		palm_dj.connect[13]=*(data_buf+17);

	 #if 1
		 for(i=0;i<14;i++){
			servo_s[i].connect=palm_dj.connect[i];
			servo_s[i].q=palm_dj.q_now[i];
		 }
	 #endif
	}	
  else if(*(data_buf+2)==0x02&&1)//FB2
  { 
 		palm_dj.connect_link=1;
		palm_dj.loss_cnt=0;
			   
		palm_dj.q_now[0]=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/CAN_POS_DIV;
		palm_dj.q_now[1]=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/CAN_POS_DIV;
		palm_dj.q_now[2]=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/CAN_POS_DIV;
		palm_dj.q_now[3]=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/CAN_POS_DIV;
		palm_dj.q_now[4]=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/CAN_POS_DIV;
		palm_dj.q_now[5]=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/CAN_POS_DIV;
		palm_dj.q_now[6]=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/CAN_POS_DIV;

	 #if 1
		 for(i=0;i<14;i++){
			servo_s[i].connect=palm_dj.connect[i];
			servo_s[i].t=palm_dj.t_now[i];
		 }
	 #endif
	}	
  else if(*(data_buf+2)==0x03&&1)//FB3
  { 
 		palm_dj.connect_link=1;
		palm_dj.loss_cnt=0;
			   
		palm_dj.q_now[7]=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/CAN_POS_DIV;
		palm_dj.q_now[8]=(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/CAN_POS_DIV;
		palm_dj.q_now[9]=(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/CAN_POS_DIV;
		palm_dj.q_now[10]=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/CAN_POS_DIV;
		palm_dj.q_now[11]=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/CAN_POS_DIV;
		palm_dj.q_now[12]=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/CAN_POS_DIV;
		palm_dj.q_now[13]=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/CAN_POS_DIV;
	 #if 1
		 for(i=0;i<14;i++){
			servo_s[i].connect=palm_dj.connect[i];
			servo_s[i].t=palm_dj.t_now[i];
		 }
	 #endif
	}			
}

u8 TxBuffer3[256];
u8 TxCounter3=0;
u8 count3=0; 
u8 Rx_Buf3[256];	//串口接收缓存
u8 RxBuffer3[255];
u8 RxState3 = 0;
u8 RxBufferNum3 = 0;
u8 RxBufferCnt3 = 0;
u8 RxLen3 = 0;
static u8 _data_len3 = 0,_data_cnt3 = 0;
void USART3_IRQHandler(void)//extcan
{ // OSIntEnter();  
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART3->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志

		com_data = USART3->DR;
		
		if(RxState3==0&&com_data==0xBA)
		{
			RxState3=1;
			RxBuffer3[0]=com_data;
		}
		else if(RxState3==1&&com_data==0xBF)
		{
			RxState3=2;
			RxBuffer3[1]=com_data;
		}
		else if(RxState3==2&&com_data>0&&com_data<0XF1)
		{
			RxState3=3;
			RxBuffer3[2]=com_data;
		}
		else if(RxState3==3&&com_data<80)
		{
			RxState3 = 4;
			RxBuffer3[3]=com_data;
			_data_len3 = com_data;
			_data_cnt3 = 0;
		}
		else if(RxState3==4&&_data_len3>0)
		{
			_data_len3--;
			RxBuffer3[4+_data_cnt3++]=com_data;
			if(_data_len3==0)
				RxState3 = 5;
		}
		else if(RxState3==5)
		{
			RxState3 = 0;
			RxBuffer3[4+_data_cnt3]=com_data;
			Data_ExtCAN_FB(RxBuffer3,_data_cnt3+5);
		}
		else
			RxState3 = 0;
	
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
				
		USART3->DR = TxBuffer2[TxCounter3++]; //写DR清除中断标志          
		if(TxCounter3 == count3)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}      
}

u8 TxBuffer4[256];
u8 TxCounter4=0;
u8 count4=0; 
u8 RxBuffer4[50];
u8 RxState4 = 0;
u8 RxBufferNum4 = 0;
u8 RxBufferCnt4 = 0;
u8 RxLen4= 0;
_FLOW optical_flow;
static u8 _data_len4 = 0,_data_cnt4 = 0;
void UART4_IRQHandler(void)//FLOW
{ //OSIntEnter(); 
	u8 com_data;
	
	if(UART4->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART4->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART4,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);//清除中断标志

		com_data = UART4->DR;
		
		if(RxState4==0)
		{
			if(com_data==0xAA){
			RxState4=1;
			RxBuffer4[0]=com_data;}
		}
		else if(RxState4==1&&com_data==0x55)
		{
			if(com_data==0x55){
			RxState4=2;
			RxBuffer4[1]=com_data;
			_data_cnt4=0;
			}else RxState4=0;
		}
		else if(RxState4==2)
		{
			RxBuffer4[2+_data_cnt4++]=com_data;
			if(_data_cnt4>5-1)
				RxState4 =3;
		}
		else if(RxState4==3)
		{
			RxBuffer4[7]=RxBuffer4[0]+RxBuffer4[1]+RxBuffer4[2]+
				RxBuffer4[3]+RxBuffer4[4]+RxBuffer4[5]+RxBuffer4[6];
      if(RxBuffer4[7]==com_data)
			{
				 optical_flow.connect=1;
			   optical_flow.loss_cnt=0;
			   optical_flow.pix[Xr]=(int8_t)RxBuffer4[2];
				 optical_flow.pix[Yr]=(int8_t)RxBuffer4[3];
				 optical_flow.height=(float)(RxBuffer4[5]+(RxBuffer4[6]<<8))/1000.;
				 optical_flow.spd_o[Xr] += optical_flow.pix[Xr];
				 optical_flow.spd_o[Yr] += optical_flow.pix[Yr];
			}
			RxState4=_data_len4=0;
		}
		else
			RxState4 = 0;
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(UART4,USART_IT_TXE ) )
	{
				
		UART4->DR = TxBuffer4[TxCounter4++]; //写DR清除中断标志          
		if(TxCounter4 == count4)
		{
			UART4->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}
	}
}


RC_GETDATA Rc_Get_PWM,Rc_Get_SBUS;
static u8 _data_len5 = 0,_data_cnt5 = 0;
void UART5_IRQHandler(void)//SBUS
{ 
	u8 com_data;
	 u16 temps;
	if(UART5->SR & USART_SR_ORE)//ORE中断
	{
		com_data = UART5->DR;
	}

  //接收中断
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志

		com_data = UART5->DR;
		oldx_sbus_rx(com_data);
		if(channels[16]==500||channels[16]==503){
		Rc_Get_SBUS.update=1;Rc_Get_SBUS.lose_cnt_rx=0;

				temps=((channels[0])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.ROLL=		 temps;
				temps=((channels[1])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.PITCH=		 temps;
				temps=((channels[2])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.THROTTLE=		 temps;
				temps=((channels[3])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.YAW=		 temps;
				temps=((channels[4])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX1=		 temps;
				temps=((channels[5])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX2=		 temps;
				temps=((channels[6])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX3=		 temps;
				temps=((channels[7])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX4=		 temps;
				temps=((channels[8])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX5=		 temps;	
				temps=((channels[9])-SBUS_MID_A)*500/((SBUS_MAX_A-SBUS_MIN_A)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.AUX6=		 temps;
				
				temps=((channels[10])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.MOV1=		 temps;
				temps=((channels[11])-SBUS_MID)*500/((SBUS_MAX-SBUS_MIN)/2)+1500;
				if(temps>900&&temps<2100)
				Rc_Get_SBUS.MOV2=		 temps;
				
				if(Rc_Get_SBUS.AUX1>1700)
					Rc_Get_SBUS.AUX_SEL1=1;
				else if(Rc_Get_SBUS.AUX1<1300)
					Rc_Get_SBUS.AUX_SEL1=-1;
				else 
					Rc_Get_SBUS.AUX_SEL1=0;
				
				if(Rc_Get_SBUS.AUX2>1700)
					Rc_Get_SBUS.AUX_SEL2=1;
				else if(Rc_Get_SBUS.AUX2<1300)
					Rc_Get_SBUS.AUX_SEL2=-1;
				else 
					Rc_Get_SBUS.AUX_SEL2=0;

				if(Rc_Get_SBUS.AUX3>1700)
					Rc_Get_SBUS.AUX_SEL3=1;
				else if(Rc_Get_SBUS.AUX3<1300)
					Rc_Get_SBUS.AUX_SEL3=-1;
				else 
					Rc_Get_SBUS.AUX_SEL3=0;

				if(Rc_Get_SBUS.AUX4>1700)
					Rc_Get_SBUS.AUX_SEL4=1;
				else if(Rc_Get_SBUS.AUX4<1300)
					Rc_Get_SBUS.AUX_SEL4=-1;
				else 
					Rc_Get_SBUS.AUX_SEL4=0;
				
				if(Rc_Get_SBUS.AUX5>1700)
					Rc_Get_SBUS.AUX_SEL5=1;
				else if(Rc_Get_SBUS.AUX5<1300)
					Rc_Get_SBUS.AUX_SEL5=-1;
				else 
					Rc_Get_SBUS.AUX_SEL5=0;
				
				if(Rc_Get_SBUS.AUX6>1700)
					Rc_Get_SBUS.AUX_SEL6=1;
				else if(Rc_Get_SBUS.AUX6<1300)
					Rc_Get_SBUS.AUX_SEL6=-1;
				else 
					Rc_Get_SBUS.AUX_SEL6=0;
				
				Rc_Get_SBUS.SBUS_CH[0]=LIMIT((float)(Rc_Get_SBUS.THROTTLE-1500)/500.0,-1,1);
				Rc_Get_SBUS.SBUS_CH[1]=LIMIT((float)(Rc_Get_SBUS.PITCH-1500)/500.0,-1,1);
				Rc_Get_SBUS.SBUS_CH[2]=LIMIT((float)(Rc_Get_SBUS.ROLL-1500)/500.0,-1,1);
				Rc_Get_SBUS.SBUS_CH[3]=LIMIT((float)(Rc_Get_SBUS.YAW-1500)/500.0,-1,1);
				Rc_Get_SBUS.SBUS_CH[4]=LIMIT((float)(Rc_Get_SBUS.MOV1-1500)/500.0,-1,1);
				Rc_Get_SBUS.SBUS_CH[5]=LIMIT((float)(Rc_Get_SBUS.MOV2-1500)/500.0,-1,1);
				
				#if 1//yunzhuo
				if(channels[16]==500){
					Rc_Get_SBUS.lose_cnt=0;
					Rc_Get_SBUS.connect=1;
				}	
				#else
				if(Rc_Get_SBUS.THROTTLE!=0){
					Rc_Get_SBUS.lose_cnt=0;
					Rc_Get_SBUS.connect=1;
				}
				#endif

		}
		if(Rc_Get_SBUS.lose_cnt_rx++>100){
		Rc_Get_SBUS.update=0;}
	}
}


void Anal_UART6(u8 *data_buf,u8 num)
{ static u8 cnt[4];
	u8 id;
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	int anal_cnt = 4;
	float temp_imu[3];
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xBA && *(data_buf+1)==0xBF))		return;		//判断帧头
	if(*(data_buf+2)==0x01&&USE_IMU_O)//
  { 
	  imuo.connect=1;
		imuo.cnt_loss=0;
		imuo.dt=Get_Cycle_T(20); 	
		imuo.att[1]=(float)((int16_t)(*(data_buf+4)<<8)|*(data_buf+5))/100.;
		imuo.att[0]=-(float)((int16_t)(*(data_buf+6)<<8)|*(data_buf+7))/100.;
		imuo.att[2]=-(float)((int16_t)(*(data_buf+8)<<8)|*(data_buf+9))/100.;
		imuo.rate[1]=(float)((int16_t)(*(data_buf+10)<<8)|*(data_buf+11))/100.;
		imuo.rate[0]=(float)((int16_t)(*(data_buf+12)<<8)|*(data_buf+13))/100.;
		imuo.rate[2]=(float)((int16_t)(*(data_buf+14)<<8)|*(data_buf+15))/100.;
		imuo.acc_b[1]=(float)((int16_t)(*(data_buf+16)<<8)|*(data_buf+17))/100./9.81;
		imuo.acc_b[0]=(float)((int16_t)(*(data_buf+18)<<8)|*(data_buf+19))/100./9.81;
		imuo.acc_b[2]=-(float)((int16_t)(*(data_buf+20)<<8)|*(data_buf+21))/100./9.81;
		temp_imu[0]=(float)((int16_t)(*(data_buf+22)<<8)|*(data_buf+23))/100.;
		temp_imu[1]=(float)((int16_t)(*(data_buf+24)<<8)|*(data_buf+25))/100.;
		temp_imu[2]=(float)((int16_t)(*(data_buf+26)<<8)|*(data_buf+27))/100.;
		
		imuo.acc_n[0]=-temp_imu[0]*sind(-imuo.att[2])+temp_imu[1]*cosd(-imuo.att[2]);
		imuo.acc_n[1]= temp_imu[0]*cosd(-imuo.att[2])+temp_imu[1]*sind(-imuo.att[2]);
		imuo.acc_n[2]= temp_imu[2];
	}
}


u8 TxBuffer6[256];
u8 TxCounter6=0;
u8 count6=0; 
u8 Rx_Buf6[256];	//串口接收缓存
u8 RxBuffer6[50];
u8 RxState6 = 0;
u8 RxBufferNum6 = 0;
u8 RxBufferCnt6 = 0;
u8 RxLen6 = 0;
static u8 _data_len6= 0,_data_cnt6 = 0;
u8 RxBuffer6_check[50];
void USART6_IRQHandler(void)//GPS----------------------
{ //OSIntEnter(); 
	u8 com_data;
	static int cnt_rx=0;
	if(USART6->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART6->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART6,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART6,USART_IT_RXNE);//清除中断标志

		com_data = USART6->DR;
		RxBuffer6_check[cnt_rx++]=com_data;
		if(cnt_rx>50)
			cnt_rx=0;
		if(RxState6==0&&com_data==0xBA)
		{
			RxState6=1;
			RxBuffer6[0]=com_data;
		}
		else if(RxState6==1&&com_data==0xBF)
		{
			RxState6=2;
			RxBuffer6[1]=com_data;
		}
		else if(RxState6==2&&com_data>0&&com_data<0XF1)
		{
			RxState6=3;
			RxBuffer6[2]=com_data;
		}
		else if(RxState6==3&&com_data<80)
		{
			RxState6 = 4;
			RxBuffer6[3]=com_data;
			_data_len6 = com_data;
			_data_cnt6 = 0;
		}
		else if(RxState6==4&&_data_len6>0)
		{
			_data_len6--;
			RxBuffer6[4+_data_cnt6++]=com_data;
			if(_data_len6==0)
				RxState6= 5;
		}
		else if(RxState6==5)
		{
			RxState6 = 0;
			RxBuffer6[4+_data_cnt6]=com_data;
			Anal_UART6(RxBuffer6,_data_cnt6+5);
   
		}
		else
			RxState6 = 0;
	}
}
u8 SendBuff1_cnt;
u8 SendBuff3_cnt;
u8 SendBuff6_cnt;
u8 SendBuff1[SEND_BUF_SIZE1];
u8 SendBuff2[SEND_BUF_SIZE2];
u8 SendBuff3[SEND_BUF_SIZE3];
u8 SendBuff6[SEND_BUF_SIZE6];
u16 leg_uart_cnt;
void GOL_LINK_TASK_DMA(void)//5ms
{
static u8 cnt[10];
static u8 flag[10];
u8 i;

end_gol_link1:;
}

void clear_leg_uart(void)
{
	u16 i;
	leg_uart_cnt=0;
	for(i=0;i<SEND_BUF_SIZE1;i++)
	SendBuff1[i]=0;
}

void Usart1_Send_DMA(u8 *dataToSend , u8 length)
{
	u8 i;
	for	(i=0;i<length;i++)
	SendBuff1[SendBuff1_cnt++]=dataToSend[i];
}

//微雪轮毂电机
//电流环模式下：-32767~32767 对应量程-8~8A(非电机电流范围)，数据类型有符号 16 位
//速度环模式下：-330~330，单位 rpm，数据类型有符号 16 位
//位置环模式下：0~32767 对应 0°~360°，数据类型无符号 16 位
char PY_CRC_8_T(char *di, uint32_t len)
{
  uint8_t crc_poly = 0x8C; //Bit sequence inversion of 0x31
	uint8_t data_t = 0; //CRC register

    for(uint32_t i = 0; i < len; i++)
    {
    	data_t ^= di[i]; //8-bit data

        for (uint8_t j = 0; j < 8; j++)
        {
            if (data_t & 0x01)
            	data_t = (data_t >> 1) ^ crc_poly;
            else
            	data_t >>= 1;
        }
    }
    return data_t;
}
 
void Set_motor_cmd(char id,int16_t cmd,int sys_mode)
{ char i;	char sum = 0;
	vs16 _temp;
  char data_to_send[50];
	char _cnt=0;
	char crc=0;
  data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=id;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0x64;
	_temp = cmd;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=BYTE0(_temp);
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;//加速模式
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=sys_mode;//制动
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	SendBuff1[SendBuff1_cnt++]=PY_CRC_8_T(data_to_send,9);
}

void Get_motor_fb(char id)
{ char i;	char sum = 0;
	vs16 _temp;
  char data_to_send[50];
	char _cnt=0;
	char crc=0;
  data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=id;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0x74;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	SendBuff1[SendBuff1_cnt++]=PY_CRC_8_T(data_to_send,9);
}
//模式值：
//0x01：设定为电流环
//0x02：设定为速度环
//0x03：设定为位置环
void Set_motor_mode(char id,char mode)
{ char i;	char sum = 0;
	vs16 _temp;
  char data_to_send[50];
	char _cnt=0;
	char crc=0;
  data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=id;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0xA0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	SendBuff1[SendBuff1_cnt++]=mode;
}

//设置ID时请保证总线上只有一个电机，每次上电只允许设置一次，电机接收到5次ID设置指令后进行设置。
void Set_motor_id(char id)
{ char i;	char sum = 0;
	vs16 _temp;
  char data_to_send[50];
	char _cnt=0;
	char crc=0;
  data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0xAA;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0x55;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0x53;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=id;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	SendBuff1[SendBuff1_cnt++]=0;
}
//电流环模式下：-32767~32767 对应量程-8~8A(非电机电流范围)，数据类型有符号 16 位
//速度环模式下：-330~330，单位 rpm，数据类型有符号 16 位
//位置环模式下：0~32767 对应 0°~360°，数据类型无符号 16 位
void set_motor_spd(char id,float rad_set){//rpm
	int16_t rpm=0;
	rpm= rad_set/(2.0 * M_PI / 60.0);
	Set_motor_cmd( id, rpm,0);
}

void set_motor_current(char id,float i_set){// 
	int16_t cmd=0;
	i_set=LIMIT(i_set,-2,2);
	cmd=i_set/8*32766/2;
	Set_motor_cmd( id, cmd,0);
}

void test_wheel(char id)
{ char i;	char sum = 0;
	vs16 _temp;
  char data_to_send[50];
	char _cnt=0;
	char crc=0;
  data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0x01;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0x64;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0x00;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0xFF;
	data_to_send[_cnt++]=SendBuff1[SendBuff1_cnt++]=0;
	SendBuff1[SendBuff1_cnt++]=PY_CRC_8_T(data_to_send,9);
}

void set_wheel_null(char len)
{   
  int i;
	for(i=0;i<len;i++)
		SendBuff1[SendBuff1_cnt++]=0x00;
}

//01 64 00 00 00 00 00 FF 00 D1 校验测试
void wheel_motor_loop(char id,float dt){
	#if 0 //1~4 1FL FR2 HL3 HR
		static int init[10]={0};
		if(!init[id]){
			//test_wheel(1);
			for(int i=0;i<15;i++)
			{
				Set_motor_id(WHEEL_ID_OFF+3);
				delay_ms(10);
			}
			
			_wheel_wx[0].rad_set=0;
			_wheel_wx[1].rad_set=0;
			_wheel_wx[2].rad_set=0;
			_wheel_wx[3].rad_set=0;
			
			_wheel_wx[0].cmd_flag=1;
			_wheel_wx[1].cmd_flag=-1;
			_wheel_wx[2].cmd_flag=1;
			_wheel_wx[3].cmd_flag=-1;
			
			_wheel_wx[id].r=0.1/2;//m 轮半径
			
			Set_motor_mode(id+WHEEL_ID_OFF,2);//设置速度环
			init[id]=1;
		}else{
		
		//Get_motor_fb(id);
			_wheel_wx[id].rad_set=LIMIT(_wheel_wx[id].rad_set,-15,15);
			set_motor_spd(id+WHEEL_ID_OFF,_wheel_wx[id].rad_set*_wheel_wx[id].cmd_flag);
		}
	#else
		static int init[10]={0};
		if(!init[id]){
		
			_wheel_wx[0].rad_set=0;
			_wheel_wx[1].rad_set=0;
			_wheel_wx[2].rad_set=0;
			_wheel_wx[3].rad_set=0;
			
			_wheel_wx[0].cmd_flag=1;
			_wheel_wx[1].cmd_flag=-1;
			_wheel_wx[2].cmd_flag=1;
			_wheel_wx[3].cmd_flag=-1;
			
			_wheel_wx[id].r=0.1/2;//m 轮半径
			_wheel_wx[id].i_2_tau=0.75;//Nm/A
			Set_motor_mode(id+WHEEL_ID_OFF,2);//设置速度环
			init[id]=1;
		}else{
			_wheel_wx[id].rad_set=LIMIT(_wheel_wx[id].rad_set,-20,20)*_wheel_wx[id].connect;
			set_motor_spd(id+WHEEL_ID_OFF,_wheel_wx[id].rad_set*_wheel_wx[id].cmd_flag*_wheel_wx[id].connect);
		}
	#endif
}

float rad_set[4]={0};
void wheel_2d_loop(float dt){
  static int init=0;
	if(!init)
	{
		init=1;
		_wheel_2d.h=0.3;//m 前后轮子间距
		_wheel_2d.w=0.303;//m  左右轮中心距离
    _wheel_2d.v_set=0;
		_wheel_2d.w_set=0;
	}

	DigitalLPF( _wheel_2d.v_set, &_wheel_2d.v_set_flt,0.6,dt);
	DigitalLPF( _wheel_2d.w_set, &_wheel_2d.w_set_flt,0.6,dt);

	float wheel_dq_exp[4]={0};
	wheel_dq_exp[0]=_wheel_2d.v_set_flt/_wheel_wx[0].r;
	wheel_dq_exp[1]=_wheel_2d.v_set_flt/_wheel_wx[1].r;
	wheel_dq_exp[2]=_wheel_2d.v_set_flt/_wheel_wx[2].r;
	wheel_dq_exp[3]=_wheel_2d.v_set_flt/_wheel_wx[3].r;

	float d_w=_wheel_2d.w_set_flt/(_wheel_2d.w/2);
 
  wheel_dq_exp[0]-=d_w;
  wheel_dq_exp[1]+=d_w;
  wheel_dq_exp[2]-=d_w;
  wheel_dq_exp[3]+=d_w;
  if(spi_master_connect_pi){//-------------连接主控采用主控轮速控制
		for(int i=0;i<4;i++)
				rad_set[i]=wheel_dq_exp[i];
	}else{//---------否则使用遥控器转速控制
		for(int i=0;i<4;i++)
				rad_set[i]=wheel_dq_exp[i];
	}

		
	DigitalLPF( rad_set[0], &_wheel_wx[0].rad_set,50,dt);
  DigitalLPF( rad_set[1], &_wheel_wx[1].rad_set,50,dt);
	DigitalLPF( rad_set[2], &_wheel_wx[2].rad_set,50,dt);
	DigitalLPF( rad_set[3], &_wheel_wx[3].rad_set,50,dt);	
	
	//-----------------state est状态估计---------------
	float temp= _wheel_wx[0].v_now
						 +_wheel_wx[1].v_now 
						 +_wheel_wx[2].v_now
						 +_wheel_wx[3].v_now;
	 
	DigitalLPF(LIMIT(temp/4, -3, 3), &_wheel_2d.v_now, 10, dt);

	float w_l=_wheel_wx[0].v_now+_wheel_wx[2].v_now;
	w_l/=2;
	float w_r=_wheel_wx[1].v_now+_wheel_wx[3].v_now;
	w_r/=2;
	float w_now=(w_r-w_l)*_wheel_2d.w;
	DigitalLPF(LIMIT(w_now, -3, 3), &_wheel_2d.w_now, 10, dt);
	
	if((_wheel_wx[0].dt_rx<0.1&&_wheel_wx[1].dt_rx<0.1&&_wheel_wx[2].dt_rx<0.1&&_wheel_wx[3].dt_rx<0.1)&&
		(_wheel_wx[0].connect&&_wheel_wx[1].connect&&_wheel_wx[2].connect&&_wheel_wx[3].connect))
		LEDRGB_ST(BLUE,1);
	else{
		LEDRGB_ST(BLUE,0);
	}
	if(spi_master_connect_pi&&spi_dt[1]<0.001)
		LEDRGB_ST(RED,1);
	else
		LEDRGB_ST(RED,0);	

}

//--------------------钟灵舵机
/*
0 末端俯仰 1末端很滚 2夹持 4云台航向 5 云台俯仰
*/

int pwm_dj[5]={1500,1500,1500,1500,1500};
int time_dj[5]={0,0,0,0,0};
void set_dj_pwm(int id,int pwm,int time){
	char cmd_return[100];//这里的cmd_return的字符长度要足够的大，根据舵机个数定，大小 = 15*个数+10
	int i=0;
	SendBuff6_cnt=0;
	pwm=LIMIT(pwm,500,2500);	
	int len=sprintf(cmd_return, "#%03dP%04dT%04d!", id, pwm, time);
	for(i=0;i<len;i++)
		SendBuff6[SendBuff6_cnt++]=cmd_return[i];
	delay_ms(1);						//延时1秒
}
		
void set_dj_pwm_all(int pwm[5],int time[5]){
	char cmd_return[100];//这里的cmd_return的字符长度要足够的大，根据舵机个数定，大小 = 15*个数+10
	char i;
	SendBuff6_cnt=0;
	for(i=0;i<5;i++)
		pwm[i]=LIMIT(pwm[i],500,2500);	
	int len=sprintf(cmd_return, "{#%03dP%04dT%04d!#%03dP%04dT%04d!#%03dP%04dT%04d!#%03dP%04dT%04d!#%03dP%04dT%04d!}", 0, pwm[0], time[0], 1, pwm[1], time[1] ,2, pwm[2], time[2]  ,4, pwm[3], time[3]  ,5, pwm[4], time[4]);
	for(i=0;i<len;i++)
		SendBuff6[SendBuff6_cnt++]=cmd_return[i];
 
	delay_ms(1);						//延时1秒
}


void dj_control_loop(float dt){
	static int init=0;
	if(!init){
		init=1;
		palm_dj.degree_to_pwm[0]=2000/180;//0 末端俯仰 1末端横滚 2夹持 4头航向 5头俯仰
		palm_dj.degree_to_pwm[1]=2000/270;//
		palm_dj.degree_to_pwm[2]=1000;
		palm_dj.degree_to_pwm[3]=2000/270;
		palm_dj.degree_to_pwm[4]=2000/270;
		palm_dj.hand_att[0]=0;//手
		palm_dj.hand_att[1]=0;
		palm_dj.head_att[0]=0;
		palm_dj.head_att[1]=0;
		palm_dj.cap_rate=0;//夹爪 0开1关
	}
	int pwm[5]={1500,1500,1500,1500,1500};
	int time[5]={0,0,0,0,0};
	pwm[0]=palm_dj.hand_att[0]*palm_dj.degree_to_pwm[0]+1500;
	pwm[1]=palm_dj.hand_att[1]*palm_dj.degree_to_pwm[1]+1500;
	pwm[2]=palm_dj.cap_rate*palm_dj.degree_to_pwm[2]+1500;
	
	pwm[3]=palm_dj.head_att[1]*palm_dj.degree_to_pwm[3]+1500;
	pwm[4]=palm_dj.head_att[0]*palm_dj.degree_to_pwm[4]+1500;
	
	//set_dj_pwm_all(pwm,time);
	set_dj_pwm(id_test,pwm_dj_test,0);
}

//-----------------------------------FT 舵机------------------
int IDs2=0;
float Pos2=0;

int Pos;
int Load;
int Current;
char test_pow=1;
_UART_SERVO servo_s[14],servo_ss[14];//0末端航向 1末端俯仰 2末端横滚 3夹持 4头航向 5头俯仰
float servo_test_q[14]={0};
#define MAX_SERVO_Q_STS3215 360.0
#define MAX_SERVO_Q_AD_STS3215 4096.0
#define END_STS3215 0

#define MAX_SERVO_Q_STS3046 360//300.0
#define MAX_SERVO_Q_AD_STS3046 4096.0
#define END_STS3046 0//处理器大小端结构

#define MAX_SERVO_Q_SCS00091 360.0
#define MAX_SERVO_Q_AD_SCS00091 1024.0
#define END_SCS00091 1//处理器大小端结构

#define MAX_SERVO_Q_SCS0009 220//300.0
#define MAX_SERVO_Q_AD_SCS0009 1024.0
#define END_SCS0009 1//处理器大小端结构

#define MAX_SERVO_Q_SCS00092 220/2//300.0
#define MAX_SERVO_Q_AD_SCS00092 1024.0
#define END_SCS00092 1//处理器大小端结构
char END_SCS_ALL=0;

uint8_t IDs[12];
uint16_t Positions[12];
int16_t Positionsm[12];
uint16_t Speeds[12];
uint8_t Accs[12];
//串口x配置为只写，CR1->RE=0, CR1->TE=1
void Uart_Flush(void)
{
	head = tail = 0;
}

int16_t Uart_Read(void)
{
	if(head!=tail){
		uint8_t Data = uartBuf[head];
		head =  (head+1)%128;
		return Data;
	}else{
		return -1;
	}
}

void Uart_Send(uint8_t *buf , uint8_t len)
{
	uint8_t i=0;
	#if USE_SERVO1
		for(i=0; i<len; i++){
			USART_SendData(USART1, buf[i]);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		}
	#elif defined(USE_SERIAL_SERVO_BOARD)||1
		for(i=0; i<len; i++){
			USART_SendData(USART2, buf[i]);
			while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		}
	#else
	sendOnlyss();
	for(i=0; i<len; i++){
		USART_SendData(USART3, buf[i]);
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	}
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	readOnlyss();
	#endif
}

#if 1//butler
void serial_init(void)//0末端航向 1末端俯仰 2末端横滚 3夹持 4头航向 5头俯仰
{
	char i,j;
	for(i=0;i<14;i++)
		servo_ss[i].bus_id=i;
 
	servo_ss[0].q_inv=-1;
	servo_ss[1].q_inv=-1;
	servo_ss[2].q_inv=-1;
	servo_ss[3].q_inv=-1;
	servo_ss[4].q_inv=-1;
	servo_ss[5].q_inv=-1;
	servo_ss[6].q_inv=-1;
	servo_ss[7].q_inv=-1;
	servo_ss[8].q_inv=-1;
	servo_ss[9].q_inv=-1;
	servo_ss[10].q_inv=1;
	servo_ss[11].q_inv=1;
	servo_ss[12].q_inv=1;
	servo_ss[13].q_inv=1;
	
	servo_ss[0].POS_OFF_SET=0;
	servo_ss[1].POS_OFF_SET=0;
	servo_ss[2].POS_OFF_SET=0;
	servo_ss[3].POS_OFF_SET=0;
	servo_ss[4].POS_OFF_SET=0;
	servo_ss[5].POS_OFF_SET=0;
	servo_ss[6].POS_OFF_SET=0;
	servo_ss[7].POS_OFF_SET=0;
	servo_ss[8].POS_OFF_SET=0;
	servo_ss[9].POS_OFF_SET=0;
	servo_ss[10].POS_OFF_SET=0;
	servo_ss[11].POS_OFF_SET=0;
	servo_ss[12].POS_OFF_SET=0;
	servo_ss[13].POS_OFF_SET=0;	
	
	palm_dj.hand_att[0]=0;//手
	palm_dj.hand_att[1]=0;
	palm_dj.head_att[0]=0;
	palm_dj.head_att[1]=0;

	palm_dj.cap_rate=0;//夹爪 0开1关  //0末端航向 1末端俯仰 2末端横滚 3夹持 4头航向 5头俯仰
	for(int i=0;i<14;i++)
		servo_ss[i].type=STS3046;
//	servo_s[0].type=STS3046;
//	servo_s[1].type=STS3046;
//	servo_s[2].type=STS3046;
//	servo_s[3].type=STS3046;
//	
//  servo_s[4].type=STS3046;
//	servo_s[5].type=STS3046;
//	servo_s[6].type=STS3046;
//	servo_s[7].type=STS3046;
//	
//	servo_s[10].type=STS3046;
//	servo_s[11].type=SCS00092;
	for(i=0;i<14;i++){
			
		
			if(servo_ss[i].type==SCS00091)
			{
				servo_ss[i].AD_Q=MAX_SERVO_Q_AD_SCS00091;
				servo_ss[i].MAX_AD_Q=MAX_SERVO_Q_SCS00091;
				servo_ss[i].END_S=END_SCS00091;
				END_SCS_ALL=END_SCS00091;
			}
			
			if(servo_ss[i].type==SCS00092)
			{
				servo_ss[i].AD_Q=MAX_SERVO_Q_AD_SCS00092;
				servo_ss[i].MAX_AD_Q=MAX_SERVO_Q_SCS00092;
				servo_ss[i].END_S=END_SCS00091;
				END_SCS_ALL=END_SCS00092;
			}
			
			if(servo_ss[i].type==SCS0009)
			{
				servo_ss[i].AD_Q=MAX_SERVO_Q_AD_SCS0009;
				servo_ss[i].MAX_AD_Q=MAX_SERVO_Q_SCS0009;
				servo_ss[i].END_S=END_SCS0009;
				END_SCS_ALL=END_SCS0009;
			}
			
					
			if(servo_ss[i].type==STS3046)
			{
				servo_ss[i].AD_Q=MAX_SERVO_Q_AD_STS3046;
				servo_ss[i].MAX_AD_Q=MAX_SERVO_Q_STS3046;
				servo_ss[i].END_S=END_STS3046;
				servo_ss[i].sms_type=1;
				END_SCS_ALL=END_STS3046;
			}
			
			if(servo_ss[i].type==STS3215)
			{
				servo_ss[i].AD_Q=MAX_SERVO_Q_AD_STS3215;
				servo_ss[i].MAX_AD_Q=MAX_SERVO_Q_STS3215;
				servo_ss[i].END_S=END_STS3215;
				servo_ss[i].sms_type=1;
				END_SCS_ALL=END_STS3215;
			}
			
			servo_ss[i].POS_OFF=servo_ss[i].AD_Q/2;
			servo_ss[i].t_measure_cmd=1;
			
		   if(Ping(servo_ss[i].bus_id)==servo_ss[i].bus_id)
				 servo_ss[i].connect=1;
			 else
				 servo_ss[i].connect=0;
	 }
}
#else//old Tinker舵机头
void serial_init(void)//0末端航向 1末端俯仰 2末端横滚 3夹持 4头航向 5头俯仰
{
	char i,j;
	servo_s[0].bus_id=0;
	servo_s[1].bus_id=1;
	servo_s[2].bus_id=2;
	servo_s[3].bus_id=3;
	servo_s[4].bus_id=4;
	servo_s[5].bus_id=5;
	servo_s[6].bus_id=6;
	servo_s[7].bus_id=7;
	servo_s[8].bus_id=8;
	servo_s[9].bus_id=9;
	servo_s[10].bus_id=10;
	servo_s[11].bus_id=11;
	
	servo_s[0].q_inv=1;
	servo_s[1].q_inv=1;
	servo_s[2].q_inv=1;
	servo_s[3].q_inv=1;
	servo_s[4].q_inv=1;
	servo_s[5].q_inv=1;
	servo_s[6].q_inv=1;
	servo_s[7].q_inv=1;
	servo_s[8].q_inv=1;
	servo_s[9].q_inv=1;
	servo_s[10].q_inv=1;
	servo_s[11].q_inv=1;
	
	servo_s[0].POS_OFF_SET=0;//2
	servo_s[1].POS_OFF_SET=0;
	servo_s[2].POS_OFF_SET=0;
	servo_s[3].POS_OFF_SET=0;
	servo_s[4].POS_OFF_SET=0;
	servo_s[5].POS_OFF_SET=0;	
	servo_s[6].POS_OFF_SET=0;//2
	servo_s[7].POS_OFF_SET=0;
	servo_s[8].POS_OFF_SET=0;
	servo_s[9].POS_OFF_SET=0;
	servo_s[10].POS_OFF_SET=0;
	servo_s[11].POS_OFF_SET=0;	


	palm_dj.hand_att[0]=0;//手
	palm_dj.hand_att[1]=0;
	palm_dj.head_att[0]=0;
	palm_dj.head_att[1]=0;

	palm_dj.cap_rate=0;//夹爪 0开1关  //0末端航向 1末端俯仰 2末端横滚 3夹持 4头航向 5头俯仰
	
	servo_s[0].type=SCS00091;
	servo_s[1].type=SCS00091;
	servo_s[2].type=SCS0009;
	servo_s[3].type=SCS0009;
	servo_s[4].type=SCS0009;
	servo_s[5].type=SCS0009;
	
	servo_s[6].type=SCS00091;
	servo_s[7].type=SCS00091;
	servo_s[8].type=SCS0009;
	servo_s[9].type=SCS0009;
	servo_s[10].type=SCS0009;
	servo_s[11].type=SCS0009;
	
	for(i=0;i<12;i++){
			if(servo_s[i].type==SCS00091)
			{
				servo_s[i].AD_Q=MAX_SERVO_Q_AD_SCS00091;
				servo_s[i].MAX_AD_Q=MAX_SERVO_Q_SCS00091;
				servo_s[i].END_S=END_SCS00091;
				END_SCS_ALL=END_SCS00091;
			}
			
			if(servo_s[i].type==SCS0009)
			{
				servo_s[i].AD_Q=MAX_SERVO_Q_AD_SCS0009;
				servo_s[i].MAX_AD_Q=MAX_SERVO_Q_SCS0009;
				servo_s[i].END_S=END_SCS0009;
				END_SCS_ALL=END_SCS0009;
			}
			
			if(servo_s[i].type==STS3215)
			{
				servo_s[i].AD_Q=MAX_SERVO_Q_AD_STS3215;
				servo_s[i].MAX_AD_Q=MAX_SERVO_Q_STS3215;
				servo_s[i].END_S=END_STS3215;
				END_SCS_ALL=END_STS3215;
			}
			
			servo_s[i].POS_OFF=servo_s[i].AD_Q/2;
			servo_s[i].t_measure_cmd=1;
			
		   if(Ping(servo_s[i].bus_id)==servo_s[i].bus_id)
				 servo_s[i].connect=1;
			 else
				 servo_s[i].connect=0;
	 }
}
#endif
void serial_ping(void)
{
		char i,j;
		for(i=0;i<14;i++){
		   if(Ping(servo_ss[i].bus_id)==servo_ss[i].bus_id)
				 servo_ss[i].connect=1;
			 else
				 servo_ss[i].connect=0;
		 }
}

void serial_ping_loss(void)
{
		char i,j;
		for(i=0;i<14;i++){
			if(servo_ss[i].connect==0){
		   if(Ping(servo_ss[i].bus_id)==servo_ss[i].bus_id)
				 servo_ss[i].connect=1;
			 else
				 servo_ss[i].connect=0;
		 }
	 }
}

void serial_power_off_all(void)
{
		unLockEprom(0xfe);//打开EPROM保存功能//卸力
		writeWord(0xfe, SCSCL_TORQUE_ENABLE, 0);
		LockEprom(0xfe);//关闭EPROM保存功能
}

void serial_power_off_joint(char id)
{
	if(servo_ss[id].connect){
		unLockEprom(id);//打开EPROM保存功能//卸力
		writeWord(id, SCSCL_TORQUE_ENABLE, 0);
		LockEprom(id);//关闭EPROM保存功能
	}
}

void serial_set_q(char id,float q_set)
{
	uint16_t POS_SET=0;
	if(servo_ss[id].connect){
		servo_ss[id].exp_q = q_set;
		
		servo_ss[id].POS_SET=POS_SET=q_set/(servo_ss[id].MAX_AD_Q/2)*servo_ss[id].AD_Q/2*servo_ss[id].q_inv+servo_ss[id].POS_OFF+servo_ss[id].POS_OFF_SET;
		if(servo_ss[id].sms_type)
			RegWritePosEx_sms(id, servo_ss[id].POS_SET, 2250, 50);
		else
			RegWritePos(id, servo_ss[id].POS_SET, 0, 1500);
	}
}

void serial_set_q_dir(char id,float q_set)
{
	uint16_t POS_SET=0;
	if(servo_ss[id].connect){
		servo_ss[id].exp_q = q_set;
		
		servo_ss[id].POS_SET=POS_SET=q_set/(servo_ss[id].MAX_AD_Q/2)*servo_ss[id].AD_Q/2*servo_ss[id].q_inv+servo_ss[id].POS_OFF+servo_ss[id].POS_OFF_SET;
		if(servo_ss[id].sms_type)
		  WritePosEx_sms(id, servo_ss[id].POS_SET, 65534, 254);
		else
			WritePos(id, servo_ss[id].POS_SET, 0, 1500);
	}
}

void serial_set_q_sync(char id,float q_set)
{
	uint16_t POS_SET=0;
	if(servo_ss[id].connect){
		servo_ss[id].exp_q = q_set;
		IDs[id]=id;
		Positionsm[id]=Positions[id]=servo_ss[id].POS_SET=POS_SET=q_set/(servo_ss[id].MAX_AD_Q/2)*servo_ss[id].AD_Q/2*servo_ss[id].q_inv+servo_ss[id].POS_OFF+servo_ss[id].POS_OFF_SET;
		Speeds[id]=2250;
		Accs[id]=50;
	}
}

void serial_servo_feed_back(char id)
{
	if(servo_ss[id].connect){
		servo_ss[id].POS = ReadPos(id);
		servo_ss[id].LOAD = ReadLoad(id);
		if(servo_ss[id].POS!=-1&&servo_ss[id].LOAD!=-1){
			servo_ss[id].q=(servo_ss[id].POS -servo_ss[id].POS_OFF)*servo_ss[id].q_inv*servo_ss[id].MAX_AD_Q/servo_ss[id].AD_Q;
			servo_ss[id].t=servo_ss[id].LOAD/1000.*servo_ss[id].t_measure_cmd;
		}
	}
}

void serial_set_q_smooth(char id,float q_set,float gain_spd,float dt)
{
	float err=servo_ss[id].exp_q-servo_ss[id].q;
	int POS_SET=0;
	if(servo_ss[id].connect){
		servo_ss[id].exp_q = q_set;
		POS_SET=(servo_ss[id].q+err*gain_spd)/(servo_ss[id].MAX_AD_Q/2)*servo_ss[id].AD_Q/2*servo_ss[id].q_inv+servo_ss[id].POS_OFF+servo_ss[id].POS_OFF_SET;
		RegWritePos(id, POS_SET, 0, 9999);
	}
}

void serial_publish_q(void){
	RegWriteAction();
}

float test_sin=500;
float cap_gain=120;
int test_q=0;
int dj_delay=20;
void serial_servo(float dt)//0末端航向 1末端俯仰 2末端横滚 3夹持 4头航向 5头俯仰
{
  static char state=0;
	static float t=0,t_ping=0;
	static int init=0;
	char i;
	
	t+=dt*test_sin;
	
	if(test_pow&&palm_dj.power==0){
		serial_power_off_all();
		delay_us(dj_delay);
		for(i=0;i<14;i++){
			serial_servo_feed_back(i);
			delay_us(dj_delay);
		}
	}else{
		if(!test_q){//测试模式
			servo_ss[0].exp_q=palm_dj.q_exp[0];
			servo_ss[1].exp_q=palm_dj.q_exp[1];
			servo_ss[2].exp_q=palm_dj.q_exp[2];
			servo_ss[3].exp_q=palm_dj.q_exp[3];
			servo_ss[4].exp_q=palm_dj.q_exp[4];
			servo_ss[5].exp_q=palm_dj.q_exp[5];
			servo_ss[6].exp_q=palm_dj.q_exp[6];
			servo_ss[7].exp_q=palm_dj.q_exp[7];
			servo_ss[8].exp_q=palm_dj.q_exp[8];
			servo_ss[9].exp_q=palm_dj.q_exp[9];
			servo_ss[10].exp_q=palm_dj.q_exp[10];
			servo_ss[11].exp_q=palm_dj.q_exp[11];
			servo_ss[12].exp_q=palm_dj.q_exp[12];
			servo_ss[13].exp_q=palm_dj.q_exp[13];
		}else{
			servo_ss[0].exp_q=servo_test_q[0];
			servo_ss[1].exp_q=servo_test_q[1];
			servo_ss[2].exp_q=servo_test_q[2];
			servo_ss[3].exp_q=servo_test_q[3];
			servo_ss[4].exp_q=servo_test_q[4];
			servo_ss[5].exp_q=servo_test_q[5];
			servo_ss[6].exp_q=servo_test_q[6];
			servo_ss[7].exp_q=servo_test_q[7];
			servo_ss[8].exp_q=servo_test_q[8];
			servo_ss[9].exp_q=servo_test_q[9];
			servo_ss[10].exp_q=servo_test_q[10];
			servo_ss[11].exp_q=servo_test_q[11];
			servo_ss[12].exp_q=servo_test_q[12];
			servo_ss[13].exp_q=servo_test_q[13];
		}

	  palm_dj.cap_touch=servo_ss[12].t;
		palm_dj.cap_touch=servo_ss[13].t;
		
#if 1//dir
		for(i=0;i<14;i++){
			serial_set_q_dir(i,servo_ss[i].exp_q);
			delay_us(dj_delay);
			serial_publish_q();
		}
#endif
	}
	
	t_ping+=dt;
	if(t_ping>10&&0){
		t_ping=0;
		serial_ping_loss();
	}
}
//------------------------------extcan
static void Send_Uart3(char *dataToSend , char length)
{
	int i;
  for(i=0;i<length;i++)
     UsartSend3(dataToSend[i]);
}

void send_cmd1_extcan(void)//extcan 扩展电机指令
{
	char i;	char sum = 0;
	char data_to_send[100];
	char _cnt=0;
	vs16 _temp;
	static END_POS pos_set;
	static float test_t=0;

  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x01;//功能字
	data_to_send[_cnt++]=0;//数据量
 	#if 0//test
	for(i=0;i<14;i++){
    palm_dj.q_exp[i]=i;
		palm_dj.q_reset[i]=i;
		palm_dj.kp[i]=i;
		palm_dj.kd[i]=i;
		palm_dj.power=1;
		palm_dj.cal_div[i]=1;
		leg_motor.motor_en=1;
  }	
	#endif	
	data_to_send[_cnt++]=palm_dj.power;

	for(i=0;i<14;i++){
		_temp = (vs16)(palm_dj.q_exp[i]*CAN_POS_DIV);
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
  }
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Uart3(data_to_send, _cnt);
}


void send_cmd2_extcan(void)//extcan 扩展电机复位与标定
{
	char i;	char sum = 0;
	char data_to_send[100];
	char _cnt=0;
	vs16 _temp;
	static END_POS pos_set;
	static float test_t=0;

  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x02;//功能字
	data_to_send[_cnt++]=0;//数据量
 
	for(i=0;i<14;i++){
		data_to_send[_cnt++]=palm_dj.cal_div[i];
  }
		
	for(i=0;i<14;i++){
		_temp = (vs16)(palm_dj.q_reset[i]*CAN_POS_DIV);
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
  }
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Uart3(data_to_send, _cnt);
}

float kp_servo[14]={15,10,10};//------------------------
float kd_servo[14]={1,0.3,0.3};
void send_cmd3_extcan(void)//extcan 扩展电机刚度
{
	char i;	char sum = 0;
	char data_to_send[100];
	char _cnt=0;
	vs16 _temp;
	static END_POS pos_set;
	static float test_t=0;

  data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x03;//功能字
	data_to_send[_cnt++]=0;//数据量
 #if 0//预设扩展电机PD参数
		for(i=0;i<14;i++){
			palm_dj.kp[i]=kp_servo[i];
			palm_dj.kd[i]=kd_servo[i];
			
			palm_dj.kp[i]*=palm_dj.stiff[i];
			palm_dj.kd[i]*=palm_dj.stiff[i];
		}
 #endif
	for(i=0;i<14;i++){
		_temp = (vs16)(palm_dj.kp[i]*palm_dj.stiff[i]*CAN_GAIN_DIV_P_M);
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
  }
	for(i=0;i<14;i++){
		_temp = (vs16)(palm_dj.kd[i]*palm_dj.stiff[i]*CAN_GAIN_DIV_D_M);
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
  }	
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Uart3(data_to_send, _cnt);
}

//play audio set
#define AUDIO_SYSTEM0 0x88
#define AUDIO_SYSTEM1 0x82
#define AUDIO_LINK 0x8c
#define AUDIO_NAG_LINK 0x8d
#define AUDIO_NUM 0xe1
#define AUDIO_POINT 0xe0
void Audio_play_ip(void)
{
	const char ipPrefix_ip[] = "192.168";
	float ipThirdPart = robot.ip1;
	float ipFourthPart = robot.ip2;
	Write_Audio_Data(0x8e);
	delay_ms(1600);
	Write_Audio_Data(AUDIO_NUM+1);delay_ms(300);
	Write_Audio_Data(AUDIO_NUM+9);delay_ms(300);
	Write_Audio_Data(AUDIO_NUM+2);delay_ms(300);
	Write_Audio_Data(AUDIO_POINT);delay_ms(300);
	Write_Audio_Data(AUDIO_NUM+1);delay_ms(300);
	Write_Audio_Data(AUDIO_NUM+6);delay_ms(300);
	Write_Audio_Data(AUDIO_NUM+8);delay_ms(300);
	Write_Audio_Data(AUDIO_POINT);delay_ms(300);
	
	if((100<=ipThirdPart)&&(ipThirdPart<=255))
	{
		Write_Audio_Data(AUDIO_NUM+(int)ipThirdPart/100%10);delay_ms(300);
		Write_Audio_Data(AUDIO_NUM+(int)ipThirdPart/10%10);delay_ms(300);
		Write_Audio_Data(AUDIO_NUM+(int)ipThirdPart%10);delay_ms(300);
		Write_Audio_Data(AUDIO_POINT);delay_ms(500);
	}
	else if((10<=ipThirdPart)&&(ipThirdPart<100))
	{
		Write_Audio_Data(AUDIO_NUM+(int)ipThirdPart/10%10);delay_ms(300);
		Write_Audio_Data(AUDIO_NUM+(int)ipThirdPart%10);delay_ms(300);
		Write_Audio_Data(AUDIO_POINT);delay_ms(500);
	}
	else
	{
		Write_Audio_Data(AUDIO_NUM+(int)ipThirdPart%10);delay_ms(300);
		Write_Audio_Data(AUDIO_POINT);delay_ms(300);
	}
	
	if((100<=ipFourthPart)&&(ipFourthPart<=255))
	{
		Write_Audio_Data(AUDIO_NUM+(int)ipFourthPart/100%10);delay_ms(300);
		Write_Audio_Data(AUDIO_NUM+(int)ipFourthPart/10%10);delay_ms(300);
		Write_Audio_Data(AUDIO_NUM+(int)ipFourthPart%10);delay_ms(300);

	}
	else if((10<=ipFourthPart)&&(ipFourthPart<100))
	{
  Write_Audio_Data(AUDIO_NUM+(int)ipFourthPart/10%10);delay_ms(300);
  Write_Audio_Data(AUDIO_NUM+(int)ipFourthPart%10);delay_ms(300);
	}
	else
	{
		Write_Audio_Data(AUDIO_NUM+(int)ipFourthPart%10);delay_ms(300);

	}
}

void Audio_system(void)
{
	//spi
	Write_Audio_Data(AUDIO_SYSTEM0);
	delay_ms(2200);
	if(robot.spi_link)
		Write_Audio_Data(AUDIO_LINK);
	else
		Write_Audio_Data(AUDIO_NAG_LINK);
	delay_ms(1200);
	//bldc extcan
	Write_Audio_Data(AUDIO_SYSTEM1);
	delay_ms(2200);
	if(robot.extcan_link)
		Write_Audio_Data(AUDIO_NUM+robot.extcan_link);
	else
		Write_Audio_Data(AUDIO_NAG_LINK);
	delay_ms(1200);
	Audio_bldc();
	delay_ms(200);
	Audio_play_ip();
}

void Audio_bldc(void)
{
	//bldc1
	Write_Audio_Data(0x83);
	delay_ms(2500);
	Write_Audio_Data(AUDIO_NUM+robot.can1_link);
	delay_ms(250);
	//bldc2
	Write_Audio_Data(0x84);
	delay_ms(2500);
	Write_Audio_Data(AUDIO_NUM+robot.can2_link);
	delay_ms(250);
}

void Write_Audio_Data(uint8_t dat)
{
	char cmd[10];
	cmd[0] = 0xAA;
	cmd[1] = 0x55;
	cmd[2] = 0xFF;
	cmd[3] = dat;  // 直接使用传入的数据
	cmd[4] = 0xFB;

	for(int j = 0; j<5 ;j++)
	{
		  UsartSend1(cmd[j]);
	}

}

//---------------------YI Dian Audio
#define FRAME_HEAD 0xFD
#define COMMAND_BYTE 0x01
#define ENCODING_GBK 0x01

typedef struct {
    uint32_t utf8_code;
    uint16_t gbk_code;
} utf8_to_gbk_mapping_t;


utf8_to_gbk_mapping_t utf8_to_gbk_mapping[] = {
    {0x00000041, 0x0041}, // A
    {0x00000042, 0x0042}, // B

    {0, 0} 
};

size_t encode_gbk(const char* utf8_str, unsigned char* out_buf, size_t out_buf_size) {
    size_t i = 0;
    size_t out_pos = 0;

    while (utf8_str[i] != '\0' && out_pos < out_buf_size) {
        uint32_t utf8_code = 0;
        uint8_t bytes_needed = 0;

        // ?? UTF-8 ??
        if ((utf8_str[i] & 0x80) == 0x00) {
            // ??? UTF-8
            utf8_code = utf8_str[i];
            bytes_needed = 1;
        } else if ((utf8_str[i] & 0xE0) == 0xC0) {
            // ??? UTF-8
            utf8_code = (utf8_str[i] & 0x1F) << 6;
            utf8_code |= (utf8_str[i + 1] & 0x3F);
            bytes_needed = 2;
        } else if ((utf8_str[i] & 0xF0) == 0xE0) {
            // ??? UTF-8
            utf8_code = (utf8_str[i] & 0x0F) << 12;
            utf8_code |= (utf8_str[i + 1] & 0x3F) << 6;
            utf8_code |= (utf8_str[i + 2] & 0x3F);
            bytes_needed = 3;
        } else {
            // ???? UTF-8 ??
            return 0;
        }

        // ????? GBK ??
        for (int j = 0; utf8_to_gbk_mapping[j].utf8_code != 0; j++) {
            if (utf8_to_gbk_mapping[j].utf8_code == utf8_code) {
                // ????? GBK ??
                if (out_pos + 2 <= out_buf_size) {
                    out_buf[out_pos++] = (utf8_to_gbk_mapping[j].gbk_code >> 8) & 0xFF;
                    out_buf[out_pos++] = utf8_to_gbk_mapping[j].gbk_code & 0xFF;
                } else {
                    // ???????
                    return 0;
                }
                break;
            }
        }

        i += bytes_needed;
    }

    return out_pos;
}

void send_custom_text(const char* text) {
    unsigned char encoded_data[512];
    size_t data_len = encode_gbk(text, encoded_data, sizeof(encoded_data));

    if (data_len == 0) {
        printf("Encoding failed.\n");
        return;
    }

    uint16_t total_data_len = (uint16_t)(data_len + 2);
    unsigned char frame[1024];
    size_t idx = 0;

    frame[idx++] = FRAME_HEAD;
    frame[idx++] = (total_data_len >> 8) & 0xFF;
    frame[idx++] = total_data_len & 0xFF;
    frame[idx++] = COMMAND_BYTE;
    frame[idx++] = ENCODING_GBK;
    memcpy(frame + idx, encoded_data, data_len);
    idx += data_len;
}

void Audio_system_yd(void)
{
 

}

