#include "can.h"
#include "ecos_link.h" 
#include "delay.h"
#include "led_fc.h"
#include "include.h"
#include "gait_math.h"
#include "locomotion_header.h"
#include "Custom_SPI_DEVICE.h"
#include "usbd_usr.h" 
#include "usbd_cdc_vcp.h" 
_LEG_MOTOR leg_motor;
motor_measure_t  motor_chassis[14];
uint32_t can1_rx_id;
int can_rx_over[5],can_rx_cnt[5];
u8 canbuft1[8],canbufr1[8];

#define USE_ID_CHECK1 0
#if CAN_NART_SEL== DISABLE || CAN_FB_SYNC //unuse
int  CAN_SAFE_DELAY=166;//us  保证各节点接收超时小于11
#else
int  CAN_SAFE_DELAY=130;//us  保证各节点接收超时小于11
#endif

u32  slave_id1 = 99 ; 
float cnt_rst1=0;
int can1_rx_cnt;
int can_rx_cnt_all[6]={0};
void CAN_motor_init(void)
{
	char i;
	
	for(i=0;i<14;i++)
	{
		leg_motor.connect=0;
		leg_motor.motor_en=0;
		leg_motor.motor_mode=MOTOR_MODE_T;	//  目前采用力矩模式
		
		reset_current_cmd(i);
		
		motor_chassis[i].max_t=leg_motor.max_t[i]=120;//Nm 最大力矩
		
		motor_chassis[i].stiff=1;
		motor_chassis[i].kp=0.5;
		motor_chassis[i].kd=0.1;
	}
}

static float fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }

static float fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }

static float Bytes2Float(unsigned char *bytes,int num)
{
    unsigned char cByte[24];
    int i;
    for (i=0;i<num;i++)
    {
		 cByte[num-1-i] = bytes[i];
    }   
    float pfValue=*(float*)&cByte;
    return  pfValue;
}

static void Float2Bytes(float pfValue,unsigned char* bytes)
{
  char* pchar=(char*)&pfValue;
  for(int i=0;i<sizeof(float);i++)
  {
    *bytes=*pchar;
     pchar++;
     bytes++;  
  }
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }

static float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
		
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,float brp,u8 mode)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	                   											 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	  
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
		#if !CAN_ABOM_E
		CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
		#else
  	CAN_InitStructure.CAN_ABOM=ENABLE;	//软件自动离线管理	  
		#endif
  	CAN_InitStructure.CAN_AWUM=ENABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=CAN_NART_SEL;//DISABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 
    
	//配置过滤器
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN1_RX0_INT_ENABLE
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
 	#if CAN_ABOM_E
		CAN_ITConfig(CAN1,CAN_IT_ERR,DISABLE);
	#endif
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);//清接收中断标志
	CAN_ClearITPendingBit(CAN1, CAN_IT_TME);//清发送中断标志  
	return 0;
}   

char can_rx_t1[10];
void CAN1_RX0_IRQHandler(void)
{
		int i;
  	CanRxMsg RxMessage;
    CAN_Receive(CAN1, 0, &RxMessage);
	
		can_rx_t1[0]=RxMessage.StdId;
	
		can_rx_t1[1]=RxMessage.Data[0];
		can_rx_t1[2]=RxMessage.Data[1];
		can_rx_t1[3]=RxMessage.Data[2];
		can_rx_t1[4]=RxMessage.Data[3];
		can_rx_t1[5]=RxMessage.Data[4];
		can_rx_t1[6]=RxMessage.Data[5];
		can_rx_t1[7]=RxMessage.Data[6];
		can_rx_t1[8]=RxMessage.Data[7];
	 
		// Frame DMG
		// CMD    | nodeID
		// 7 bits | 4 bits
		uint32_t frameID = RxMessage.StdId;
		uint32_t cmd = (frameID >> 4);
		uint32_t nodeID = (frameID & 0xF)-1;
		
		//----------------------------MIT  电机CAN1 ID从1开始  双足：CAN1对应左腿  四足对应：左前 右前
		if ((motor_chassis[0].motor.type<EC_1)&&
		(RxMessage.IDE == CAN_Id_Standard) //标准帧、
		&& (RxMessage.IDE == CAN_RTR_Data) //数据帧、
		&& ((RxMessage.DLC == 6||RxMessage.DLC == 8))) /* 数据长度为8 */
		{
			if(RxMessage.Data[0]==0+1||RxMessage.Data[0]==0+1+0x10){
				data_can_mit_anal(&motor_chassis[0],RxMessage.Data);
				leg_motor.q_now[0]=motor_chassis[0].q_now_flt;
				leg_motor.qd_now[0]=motor_chassis[0].qd_now_flt;
				leg_motor.t_now[0]=motor_chassis[0].t_now_flt;
			}
			else if(RxMessage.Data[0]==1+1||RxMessage.Data[0]==1+1+0x10){
				data_can_mit_anal(&motor_chassis[1],RxMessage.Data);
				leg_motor.q_now[1]=motor_chassis[1].q_now_flt;
				leg_motor.qd_now[1]=motor_chassis[1].qd_now_flt;
				leg_motor.t_now[1]=motor_chassis[1].t_now_flt;
			}
			else if(RxMessage.Data[0]==2+1||RxMessage.Data[0]==2+1+0x10){
				data_can_mit_anal(&motor_chassis[2],RxMessage.Data);
				leg_motor.q_now[2]=motor_chassis[2].q_now_flt;
				leg_motor.qd_now[2]=motor_chassis[2].qd_now_flt;
				leg_motor.t_now[2]=motor_chassis[2].t_now_flt;
			}
			else if(RxMessage.Data[0]==3+1||RxMessage.Data[0]==3+1+0x10){
				data_can_mit_anal(&motor_chassis[3],RxMessage.Data);
				leg_motor.q_now[3]=motor_chassis[3].q_now_flt;
				leg_motor.qd_now[3]=motor_chassis[3].qd_now_flt;
				leg_motor.t_now[3]=motor_chassis[3].t_now_flt;
			}
			else if(RxMessage.Data[0]==4+1||RxMessage.Data[0]==4+1+0x10){
				data_can_mit_anal(&motor_chassis[4],RxMessage.Data);
				leg_motor.q_now[4]=motor_chassis[4].q_now_flt;
				leg_motor.qd_now[4]=motor_chassis[4].qd_now_flt;
				leg_motor.t_now[4]=motor_chassis[4].t_now_flt;
			}
		  else if(RxMessage.Data[0]==5+1||RxMessage.Data[0]==5+1+0x10){
				data_can_mit_anal(&motor_chassis[5],RxMessage.Data);
				leg_motor.q_now[5]=motor_chassis[5].q_now_flt;
				leg_motor.qd_now[5]=motor_chassis[5].qd_now_flt;
				leg_motor.t_now[5]=motor_chassis[5].t_now_flt;
			}
		 else if(RxMessage.Data[0]==6+1||RxMessage.Data[0]==6+1+0x10){
				data_can_mit_anal(&motor_chassis[6],RxMessage.Data);
				leg_motor.q_now[6]=motor_chassis[6].q_now_flt;
				leg_motor.qd_now[6]=motor_chassis[6].qd_now_flt;
				leg_motor.t_now[6]=motor_chassis[6].t_now_flt;
			}
		}	
		
				//----------------------------RV系列 VESC驱动
		if (
		(RxMessage.IDE == CAN_Id_Standard) //标准帧、
		&& (RxMessage.IDE == CAN_RTR_Data) //数据帧、
		&& ((RxMessage.DLC >=6)) &&
			motor_chassis[0].motor.type>=EC_1) /* 数据长度为8 */
		{
			RV_can_data_repack(&RxMessage,0,0);
			for(i=0;i<14;i++)
			{
				leg_motor.q_now[i]=motor_chassis[i].q_now_flt;
				leg_motor.qd_now[i]=motor_chassis[i].qd_now_flt;
				leg_motor.t_now[i]=motor_chassis[i].t_now_flt;
			}
		}			
		
	  can1_rx_cnt++;
}
 

u8 CAN1_Send_Msg(u8* msg,u8 len,uint32_t id)
{	
	static char cnt_tx;
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12;	 // 标准标识符为0
  TxMessage.ExtId=0x00;//0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)
			return 1;
  return 0;		//good

}

u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
	static int cnt_rx,led_flag;
	char can_node_id=0;
	char temp;
	int id=0;
	CAN_Receive(CAN1, 0, &RxMessage);
	cnt_rst1=0;
	
	if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
	for(i=0;i<RxMessage.DLC;i++)
	buf[i]=RxMessage.Data[i]; 
	can1_rx_id=RxMessage.StdId;	
	can1_rx_cnt++;
	return RxMessage.DLC;	
}


uint32_t can2_rx_id;
u8 canbuft2[8];
u8 canbufr2[8];
#define USE_ID_CHECK2 0
u32  slave_id2 = 99 ; 
float cnt_rst2;
int can2_rx_cnt;
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,float brp,u8 mode)
{
  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN2_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif
    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2); //GPIOA11复用为CAN1
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2); //GPIOA12复用为CAN1
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
		#if !CAN_ABOM_E
		CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
		#else
  	CAN_InitStructure.CAN_ABOM=ENABLE;	//软件自动离线管理	  
		#endif
  	CAN_InitStructure.CAN_AWUM=ENABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=CAN_NART_SEL;//DISABLE	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN1 
    
	//配置过滤器
 	  CAN_FilterInitStructure.CAN_FilterNumber=14;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		
#if CAN2_RX0_INT_ENABLE
	  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif

	#if CAN_ABOM_E
		CAN_ITConfig(CAN2,CAN_IT_ERR,DISABLE);
	#endif
	return 0;
}   
 
#if CAN2_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN2_RX0_IRQHandler(void)
{
	int ge,shi,bai;
	static int cnt_rx,led_flag,id=0;
	CanRxMsg RxMessage;
	int i=0;
	u8 temp;
	CAN_Receive(CAN2, 0, &RxMessage);
	cnt_rst2=0;
//	if(cnt_rx++>500&&1){cnt_rx=0;
//	led_flag=!led_flag;
//	//LEDRGB_BLUE(led_flag);
//	}
//	for(i=0;i<8;i++)
//		canbufr2[i]=0x00;
//	for(i=0;i<RxMessage.DLC;i++)
//		canbufr2[i]=RxMessage.Data[i];
//	can2_rx_id=RxMessage.StdId;	
// 
	//----------------------------MIT电机CAN2 ID从1开始  双足：CAN2对应右腿  四足对应：左后 右后
	if ((motor_chassis[0].motor.type<EC_1)&& 
	(RxMessage.IDE == CAN_Id_Standard) //标准帧、
	&& (RxMessage.IDE == CAN_RTR_Data) //数据帧、
	&& ((RxMessage.DLC == 6||RxMessage.DLC == 8))) /* 数据长度为8 */
	{
			if(RxMessage.Data[0]==0+1||RxMessage.Data[0]==0+1+0x10){
				data_can_mit_anal(&motor_chassis[7+0],RxMessage.Data);
				leg_motor.q_now[7+0]=motor_chassis[7+0].q_now_flt;
				leg_motor.qd_now[7+0]=motor_chassis[7+0].qd_now_flt;
				leg_motor.t_now[7+0]=motor_chassis[7+0].t_now_flt;
			}
			else if(RxMessage.Data[0]==1+1||RxMessage.Data[0]==1+1+0x10){
				data_can_mit_anal(&motor_chassis[7+1],RxMessage.Data);
				leg_motor.q_now[7+1]=motor_chassis[7+1].q_now_flt;
				leg_motor.qd_now[7+1]=motor_chassis[7+1].qd_now_flt;
				leg_motor.t_now[7+1]=motor_chassis[7+1].t_now_flt;
			}
			else if(RxMessage.Data[0]==2+1||RxMessage.Data[0]==2+1+0x10){
				data_can_mit_anal(&motor_chassis[7+2],RxMessage.Data);
				leg_motor.q_now[7+2]=motor_chassis[7+2].q_now_flt;
				leg_motor.qd_now[7+2]=motor_chassis[7+2].qd_now_flt;
				leg_motor.t_now[7+2]=motor_chassis[7+2].t_now_flt;
			}
			else if(RxMessage.Data[0]==3+1||RxMessage.Data[0]==3+1+0x10){
				data_can_mit_anal(&motor_chassis[7+3],RxMessage.Data);
				leg_motor.q_now[7+3]=motor_chassis[7+3].q_now_flt;
				leg_motor.qd_now[7+3]=motor_chassis[7+3].qd_now_flt;
				leg_motor.t_now[7+3]=motor_chassis[7+3].t_now_flt;
			}
			else if(RxMessage.Data[0]==4+1||RxMessage.Data[0]==4+1+0x10){
				data_can_mit_anal(&motor_chassis[7+4],RxMessage.Data);
				leg_motor.q_now[7+4]=motor_chassis[7+4].q_now_flt;
				leg_motor.qd_now[7+4]=motor_chassis[7+4].qd_now_flt;
				leg_motor.t_now[7+4]=motor_chassis[7+4].t_now_flt;
			}
		  else if(RxMessage.Data[0]==5+1||RxMessage.Data[0]==5+1+0x10){
				data_can_mit_anal(&motor_chassis[7+5],RxMessage.Data);
				leg_motor.q_now[7+5]=motor_chassis[7+5].q_now_flt;
				leg_motor.qd_now[7+5]=motor_chassis[7+5].qd_now_flt;
				leg_motor.t_now[7+5]=motor_chassis[7+5].t_now_flt;
			}
		 else if(RxMessage.Data[0]==6+1||RxMessage.Data[0]==6+1+0x10){
				data_can_mit_anal(&motor_chassis[7+6],RxMessage.Data);
				leg_motor.q_now[7+6]=motor_chassis[7+6].q_now_flt;
				leg_motor.qd_now[7+6]=motor_chassis[7+6].qd_now_flt;
				leg_motor.t_now[7+6]=motor_chassis[7+6].t_now_flt;
			}
	}	
	
		//----------------------------RV系列 VESC驱动
		if (
		(RxMessage.IDE == CAN_Id_Standard) //标准帧、
		&& (RxMessage.IDE == CAN_RTR_Data) //数据帧、
		&& ((RxMessage.DLC >=6)) &&
			motor_chassis[0].motor.type>=EC_1) /* 数据长度为8 */
		{
			RV_can_data_repack(&RxMessage,0,1);//can2
			for(i=0;i<14;i++){
				leg_motor.q_now[i]=motor_chassis[i].q_now_flt;
				leg_motor.qd_now[i]=motor_chassis[i].qd_now_flt; 
				leg_motor.t_now[i]=motor_chassis[i].t_now_flt;
			}
		}			
		
		can2_rx_cnt++;
}
#endif


u8 CAN2_Send_Msg(u8* msg,u8 len,uint32_t id)
{	
	static char cnt_tx;
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12;	 // 标准标识符为0
  TxMessage.ExtId=0x00;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN2, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)
			return 1;
  return 0;		//good
}

u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0;i<RxMessage.DLC;i++)
    buf[i]=RxMessage.Data[i];  
	  can2_rx_id=RxMessage.StdId;	
	return RxMessage.DLC;	
}
//----------------------------------------------------------------------------------------------------------------------
void reset_current_cmd(char id)//清除电流
{
	leg_motor.set_t[id]=0;
	leg_motor.set_i[id]=0;
}

u8 CAN1_Send_Msg_Board(u8* msg,u8 len,uint32_t id)
{	
	static char cnt_tx;
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12;	 // 标准标识符为0
  TxMessage.ExtId=0x00;//0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=0;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=0;				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)
			return 1;
  return 0;		//good
}

u8 CAN2_Send_Msg_Board(u8* msg,u8 len,uint32_t id)
{	
	static char cnt_tx;
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;//0x12;	 // 标准标识符为0
  TxMessage.ExtId=0x00;//0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=0;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=0;				 // 第一帧信息          
  mbox= CAN_Transmit(CAN2, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)
			return 1;
  return 0;		//good
}

void CAN_motor_sm(float dt)
{
	char i=0;
	char id_cnt=0;
	
	if(motor_chassis[0].motor.type>=EC_1)//----------ECOS电机 motor.type 是读取的上位机配置
	{
		if(!can_cmd_usb_disable||ocu_connect==0)//usb没连接
		{
			for(i=0;i<14;i++)
				motor_chassis[i].en_cmd=leg_motor.motor_en;
				mit_bldc_thread_rv(leg_motor.motor_en,dt);
		}
		else
		{
			for(i=0;i<14;i++)
			{
				motor_chassis[i].en_cmd=motor_chassis[i].en_cmd_ocu;
				mit_bldc_thread_rv(motor_chassis[0].en_cmd_ocu,dt);
			}
		}
	}
	else	//---------------------------------------------DM电机
	{
		if(!can_cmd_usb_disable||ocu_connect==0)//usb没连接
		{
			for(i=0;i<14;i++)
				motor_chassis[i].en_cmd=leg_motor.motor_en;
				mit_bldc_thread(leg_motor.motor_en,dt);
		}
		else
		{
			for(i=0;i<14;i++)
			{
				motor_chassis[i].en_cmd=motor_chassis[i].en_cmd_ocu;
				mit_bldc_thread(motor_chassis[0].en_cmd_ocu,dt);
			}
		}
	}
}

