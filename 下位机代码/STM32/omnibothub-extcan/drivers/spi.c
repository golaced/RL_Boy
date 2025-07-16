
#include "spi.h"
#include "locomotion_header.h"
#include "can.h"
#include "dog.h"
#include "led_fc.h"
#include "usart_fc.h"
#include "usbd_cdc_vcp.h" 
#include "Custom_SPI_DEVICE.h"
void SPI3_Init(void)
{	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; //ce
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_15; //csn  pa4
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	
	SPI_CS(NRF2401,1);
	SPI_CS(MPU9250,1);
	SPI_CS(CS_FLASH,1);
	SPI_CS(MS5611,1);
	SPI_CS(CS_LIS,1);
	

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3);
 

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);
         	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI3, &SPI_InitStructure); 
	/* Enable SPI1 */ 
	SPI_Cmd(SPI3, ENABLE);
}

void SPI_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
	SPI3->CR1&=0XFFC7;//λ3-5���㣬�������ò�����
	SPI3->CR1|=SPI_BaudRatePrescaler;	//����SPI1�ٶ� 
	SPI_Cmd(SPI3,ENABLE); //ʹ��SPI1
} 

u8 Spi_RW(u8 dat) 
{ 

	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET); 

	SPI_I2S_SendData(SPI3, dat); 

	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET); 
	/* Return the byte read from the SPI bus */ 
	return SPI_I2S_ReceiveData(SPI3); 
}


void SPI_Receive(uint8_t *pData, uint16_t Size)
{
    for(uint16_t i=0; i<Size; i++)
    {
        pData[i] = Spi_RW(0);
    }
}

void SPI_CS(u8 sel,u8 set)
{

	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
	GPIO_SetBits(GPIOA, GPIO_Pin_15);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
	delay_us(10);
switch(sel)
{
	case MPU9250:
  if(set)	
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	else
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	//delay_us(10);
	break;
	case NRF2401:
	if(set)	
	GPIO_SetBits(GPIOC, GPIO_Pin_4);
	else
	GPIO_ResetBits(GPIOC, GPIO_Pin_4);
	//delay_us(10);
	break;
	case MS5611:
	if(set)	
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	else
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	break;	
	case CS_FLASH:
	if(set)	
	GPIO_SetBits(GPIOA, GPIO_Pin_8);
	else
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	break;
	case CS_LIS:
	if(set)	
	GPIO_SetBits(GPIOA, GPIO_Pin_15);
	else
	GPIO_ResetBits(GPIOA, GPIO_Pin_15);
	//delay_us(10);
	break;
}
}	

//-----------------------
#define DMA_SPI2        0
#define RX_LEN 		    	(uint8_t)64
#define TX_LEN          (uint8_t)64

#define SPI1_DR_ADDR    (uint32_t)(&SPI2->DR)
uint8_t SPI_RX_BUFFER[RX_LEN]= {0,}; 
uint8_t SPI_TX_BUFFER[TX_LEN]= {0x1,0x2,0x3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22}; 


static void spi_dma_init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
	/* DMA RX config */
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;                                     // DMA  ͨ�� RX
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_ADDR;                   //  �����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI_RX_BUFFER;     // ���ջ��������ڴ��е���һ�����飩
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                    //DMA ���䷽��
	DMA_InitStructure.DMA_BufferSize = RX_LEN;                                                     //  DMA ���������    ������ڻ������ٸ�
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;             //  �����ַ����  ȡ��
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    // �ڴ��ַ����  ʹ��
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;     //  ����� ��λ ��byte  8bit��
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;// ����� ��λ ��byte  8bit��
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //  ��ͨģʽ  �������һ�ξ��Զ�����
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                              // ���ȼ� �е�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                   //��ʹ�� FIFO
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;             //
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;              //
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;        //
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);                                            //��ʼ��
	                                                                                                                   //
	/* DMA TX Config */                                                                                      //
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                     // DMA  ͨ�� TX
	DMA_InitStructure.DMA_PeripheralBaseAddr = SPI1_DR_ADDR;                   //   �����ַ 
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SPI_TX_BUFFER;     // ���ջ��������ڴ��е���һ�����飩
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                    // DMA ���䷽�� 
	DMA_InitStructure.DMA_BufferSize = TX_LEN;                                                     //  DMA ���������    ������ڻ������ٸ�
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;             //  �����ַ����  ȡ��
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    // �ڴ��ַ����  ʹ��
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;     //   ����� ��λ ��byte  8bit�� 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;// ����� ��λ ��byte  8bit��
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //   ��ͨģʽ  �������һ�ξ��Զ����� 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                                   //   ��ͨģʽ  �������һ�ξ��Զ����� 
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                              // ���ȼ� �е�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                   //
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;             //
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;              //
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;        //
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);                                            //��ʼ��
}


void SPI2_Init(void)
{	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	NVIC_InitTypeDef   NVIC_InitStructure;

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);
 

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);
         	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��

	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;

	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;    // CPOL = 0  PI
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;  // CPHA = 0

	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;	
//SPI_SPEED_2   2??   (SPI 12M    --sys 24M)
//SPI_SPEED_8   8??   (SPI 3M     --sys 24M)
//SPI_SPEED_16  16??  (SPI 1.5M    --sys 24M)
//SPI_SPEED_256 256?? (SPI  905.6K --sys 24M)
	SPI_InitStructure.SPI_BaudRatePrescaler = SLAVE_SPI_BAUDRATE;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI_InitStructure); 
#if DMA_SPI2
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);  
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
	spi_dma_init();
	
	NVIC_InitStructure.NVIC_IRQChannel         = DMA1_Stream3_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd      = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);

	SPI_Cmd(SPI2, ENABLE);
	SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);
#else
	NVIC_InitStructure.NVIC_IRQChannel 						= SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE);
	SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_RXNE, ENABLE);	
	//SPI_SSOutputCmd(SPI2, DISABLE);
	SPI_Cmd(SPI2, ENABLE); //ʹ��SPI����
	#endif
}



static __IO uint32_t  TimeOut = SPIT_LONG_TIMEOUT; 
int spi_master_connect_pi=0,spi_master_loss_pi_all=0;
int spi_master_loss_pi=0;
int spi_comm_mess_type=0;
unsigned char  spi_tx_buf[SPI_BUF_SIZE]={0};
unsigned char  spi_rx_buf[SPI_BUF_SIZE]={0};
int spi_tx_cnt=0;
int spi_rx_cnt=0;
int spi_rx_cnt_all=0;
char spi_flag_pi[2]={0,0};
float spi_dt[10]={0};
uint8_t SPI_Send_Byte(u16 data)
{
	TimeOut = SPIT_LONG_TIMEOUT;
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET)
	{}
	SPI_I2S_SendData(SPI2,data);
	TimeOut = SPIT_LONG_TIMEOUT;
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET)
	{}
	return SPI_I2S_ReceiveData(SPI2);
}

static void setDataInt_spi(int i)
{
	spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
	spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static void setDataFloat_spi(float f)
{
	int i = *(int *)&f;
	spi_tx_buf[spi_tx_cnt++] = ((i << 24) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 16) >> 24);
	spi_tx_buf[spi_tx_cnt++] = ((i << 8) >> 24);
	spi_tx_buf[spi_tx_cnt++] = (i >> 24);
}

static void setDataFloat_spi_int(float f,float size)
{
	vs16 _temp;
	_temp=f*size;
	spi_tx_buf[spi_tx_cnt++] = BYTE1(_temp);
	spi_tx_buf[spi_tx_cnt++] = BYTE0(_temp);
}


static float floatFromData_spi(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=4;
	return *(float *)&i;
}

static float floatFromData_spi_int(unsigned char *data, int *anal_cnt,float size)
{
	float temp=0;
	temp=(float)((int16_t)(*(data + *anal_cnt + 0)<<8)|*(data + *anal_cnt + 1))/size;
	*anal_cnt += 2;
	return temp;
}

static char charFromData_spi(unsigned char *data,int* anal_cnt)
{
	int temp=*anal_cnt ;
	*anal_cnt +=1;
	return *(data+temp);
}

static int intFromData_spi(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	*anal_cnt +=4;
	return i;
}

//for slave
void slave_send(char sel)//���͵�Odroid
{
  int i;
	char id=0;
	char sum_t=0,_cnt=0;

	spi_tx_cnt=0;
	
	spi_tx_buf[spi_tx_cnt++]=0xFF;
	spi_tx_buf[spi_tx_cnt++]=0xFB;
	spi_tx_buf[spi_tx_cnt++]=sel;
	spi_tx_buf[spi_tx_cnt++]=0;
	switch(sel)
	{
		case 26://STM32�����巢�͵�Odrid  tinker human

				setDataFloat_spi(robotwb.now_att.pitch);
				setDataFloat_spi(robotwb.now_att.roll);
				setDataFloat_spi(robotwb.now_att.yaw);	
			
				setDataFloat_spi(robotwb.now_rate.pitch);
				setDataFloat_spi(robotwb.now_rate.roll);
				setDataFloat_spi(robotwb.now_rate.yaw);	
			
				setDataFloat_spi(vmc_all.acc_b.x);//������ٶ�
				setDataFloat_spi(vmc_all.acc_b.y);
				setDataFloat_spi(vmc_all.acc_b.z);

			for(id=0;id<14;id++){
				setDataFloat_spi_int(leg_motor.q_now[id],CAN_POS_DIV);//�ؽڽǶ�
				setDataFloat_spi_int(leg_motor.qd_now[id],CAN_DPOS_DIV);//�ؽڽ��ٶ�
				setDataFloat_spi_int(leg_motor.t_now[id],CAN_T_DIV);//�ؽ�Ť��
				spi_tx_buf[spi_tx_cnt++]=100+(leg_motor.connect_motor[id]*10+leg_motor.ready[id]);//����״̬
			}		
		break;
		case 36://STM32�����巢�͵�Odrid  tinker human
				spi_tx_buf[spi_tx_cnt++]=ocu.connect;
				spi_tx_buf[spi_tx_cnt++]=ocu.key_st;
				spi_tx_buf[spi_tx_cnt++]=ocu.key_back;
				spi_tx_buf[spi_tx_cnt++]=ocu.key_lr;//1
		
				spi_tx_buf[spi_tx_cnt++]=ocu.key_ud;
				spi_tx_buf[spi_tx_cnt++]=ocu.key_x;
				spi_tx_buf[spi_tx_cnt++]=ocu.key_a;
				spi_tx_buf[spi_tx_cnt++]=ocu.key_b;//2
		
				spi_tx_buf[spi_tx_cnt++]=ocu.key_y;
				spi_tx_buf[spi_tx_cnt++]=ocu.key_ll;
				spi_tx_buf[spi_tx_cnt++]=ocu.key_rr;		
				
				setDataFloat_spi_int(ocu.rc_spd_w[Xr],100);//3
				setDataFloat_spi_int(ocu.rc_spd_w[Yr],100);
				
				setDataFloat_spi_int(ocu.rc_att_w[PITr],100);//4
				setDataFloat_spi_int(ocu.rc_att_w[ROLr],100);
				
				setDataFloat_spi_int(ocu.rate_yaw_w,100);
				
				for(id=0;id<15;id++)
					spi_tx_buf[spi_tx_cnt++]=0;			
					
			for(id=0;id<14;id++){
				setDataFloat_spi_int(servo_s[id].q,CAN_POS_DIV);//�ؽڽǶ�
				setDataFloat_spi_int(leg_motor.qd_now[id],CAN_DPOS_DIV);//�ؽڽ��ٶ�
				setDataFloat_spi_int(servo_s[id].t,CAN_T_DIV);//�ؽ�Tau
				spi_tx_buf[spi_tx_cnt++]=servo_s[id].connect;//����״̬
			}		
		break;
	}
	spi_tx_buf[3] =(spi_tx_cnt)-4;
		for( i=0;i<spi_tx_cnt;i++)
	sum_t += spi_tx_buf[i];
	spi_tx_buf[spi_tx_cnt++] = sum_t;
	
}

float test_spi_rx[2]={0};
int sum_spi_err=0;
int temp_sel[4][128]={0};
void slave_rx(u8 *data_buf,u8 num)//---------------------------��Linux����������ָ��
{ static u8 cnt[4];
	u8 id;
	char temp_char;
	vs16 rc_value_temp;
	u8 sum = 0;
	u8 i;
	int anal_cnt=4;
	float kp_sw,ki_sw,kd_sw;
	float kp_st,ki_st,kd_st;
	char bldc_id_sel=0;
	
	for( i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		{
		sum_spi_err++;
		return;}		//�ж�sum
	if(!(*(data_buf)==0xFE && *(data_buf+1)==0xFC))		return;		//�ж�֡ͷ
  if(*(data_buf+2)==45)//���ص��������==============tinker human BLDC
  { 
		spi_comm_mess_type=SPI_MESS_TYPE_3BLDC_DIV;
		spi_dt[1] = Get_Cycle_T(17); 
	  spi_master_loss_pi=0;
		spi_master_connect_pi=1;
		IWDG_Feed();
		spi_rx_cnt_all++;
		
		rc_value_temp=charFromData_spi(spi_rx_buf,&anal_cnt);
		leg_motor.motor_en=rc_value_temp/100;//���ʹ��
		leg_motor.reset_q=(rc_value_temp-leg_motor.motor_en*100)/10;//�����0��־λ
		leg_motor.reset_err=rc_value_temp%10;//��λ����

		rc_value_temp=charFromData_spi(spi_rx_buf,&anal_cnt);
		mems.Acc_CALIBRATE=rc_value_temp/100;//�궨���ٶȼ�
		mems.Gyro_CALIBRATE=(rc_value_temp-mems.Acc_CALIBRATE*100)/10;//�궨������
		mems.Mag_CALIBRATE=rc_value_temp%10;//�궨�ų�
		
    robotwb.beep_state=charFromData_spi(spi_rx_buf,&anal_cnt);//������״̬		
		
		for(i=0;i<14;i++){
			leg_motor.q_set[i]=floatFromData_spi_int(spi_rx_buf,&anal_cnt,CAN_POS_DIV);//�����Ƕ�
			leg_motor.set_t[i]=floatFromData_spi_int(spi_rx_buf,&anal_cnt,CAN_T_DIV);//����Ť��
			leg_motor.kp[i]=floatFromData_spi_int(spi_rx_buf,&anal_cnt,CAN_GAIN_DIV_P);
			leg_motor.kd[i]=floatFromData_spi_int(spi_rx_buf,&anal_cnt,CAN_GAIN_DIV_D);		
		}
			

//----------------�������������---------------------
		if(ocu_connect==0||1){
			for(i=0;i<14;i++){
				motor_chassis[i].set_q=leg_motor.q_set[i]; 
			  if(motor_chassis[i].param.control_mode==1)	
					motor_chassis[i].set_qd=leg_motor.q_set[i]; 
				else
					motor_chassis[i].set_qd=0;//Ԥ��
				motor_chassis[i].set_t=leg_motor.set_t[i]; 
				motor_chassis[i].param.q_reset_angle=leg_motor.q_reset[i];
				motor_chassis[i].kp=leg_motor.kp[i]; 
				motor_chassis[i].kd=leg_motor.kd[i]; 	
				motor_chassis[i].max_t=leg_motor.max_t[i]; 
				motor_chassis[i].stiff=leg_motor.stiff[i]; 
				if(leg_motor.reset_q==2&&motor_chassis[i].reset_q==0)
					motor_chassis[i].reset_q=1;
				motor_chassis[i].param.usb_cmd_mode=1;//����Ϊλ��ģʽ
			}
		}
	}
 if(*(data_buf+2)==55)//���ص��������==============tinker human DJ
  { 
		spi_comm_mess_type=SPI_MESS_TYPE_3BLDC_DIV;
		spi_dt[2] = Get_Cycle_T(18); 
	  spi_master_loss_pi=0;
		spi_master_connect_pi=1;
		IWDG_Feed();
		spi_rx_cnt_all++;
		
		rc_value_temp=charFromData_spi(spi_rx_buf,&anal_cnt);
		palm_dj.power=rc_value_temp;//���ʹ��

		rc_value_temp=charFromData_spi(spi_rx_buf,&anal_cnt);
		mems.Acc_CALIBRATE=rc_value_temp/100;//�궨���ٶȼ�
		mems.Gyro_CALIBRATE=(rc_value_temp-mems.Acc_CALIBRATE*100)/10;//�궨������
		mems.Mag_CALIBRATE=rc_value_temp%10;//�궨�ų�
		
    robotwb.beep_state=charFromData_spi(spi_rx_buf,&anal_cnt);//������״̬		
		
		for(i=0;i<14;i++){
			palm_dj.q_exp[i]=floatFromData_spi_int(spi_rx_buf,&anal_cnt,CAN_POS_DIV);//��0�Ƕ�
			leg_motor.q_reset[i]=floatFromData_spi_int(spi_rx_buf,&anal_cnt,CAN_POS_DIV);//��0�Ƕ�
			leg_motor.stiff[i]=floatFromData_spi_int(spi_rx_buf,&anal_cnt,CAN_F_DIV);//������
			leg_motor.cal_div[i]=charFromData_spi(spi_rx_buf,&anal_cnt);//������
		}
				
//----------------�������������---------------------
		if(ocu_connect==0||1){
			for(i=0;i<14;i++){
			  if(motor_chassis[i].param.control_mode==1)	
					motor_chassis[i].set_qd=leg_motor.q_set[i]; 
				else
					motor_chassis[i].set_qd=0;//Ԥ��
				motor_chassis[i].set_t=leg_motor.set_t[i];
				motor_chassis[i].param.q_reset_angle=leg_motor.q_reset[i];
				motor_chassis[i].kp=leg_motor.kp[i]; 
				motor_chassis[i].kd=leg_motor.kd[i]; 	
				motor_chassis[i].max_t=leg_motor.max_t[i]; //���Ť����������λ������
				motor_chassis[i].stiff=leg_motor.stiff[i]; 
				if(leg_motor.reset_q==2&&motor_chassis[i].reset_q==0)
					motor_chassis[i].reset_q=1;
				motor_chassis[i].param.usb_cmd_mode=1;//����Ϊλ��ģʽ
				motor_chassis[i].cal_div=leg_motor.cal_div[i];//�����궨ģʽ
			}
		}
	}
		else if (*(data_buf+2)==50)//OCU mems
  {
		mems.imu_pos.x=floatFromData_spi_int(spi_rx_buf,&anal_cnt,1000);	
		mems.imu_pos.y=floatFromData_spi_int(spi_rx_buf,&anal_cnt,1000);	
		mems.imu_pos.z=floatFromData_spi_int(spi_rx_buf,&anal_cnt,1000);	
		mems.imu_att.x=floatFromData_spi_int(spi_rx_buf,&anal_cnt,CAN_POS_DIV);	
		mems.imu_att.y=floatFromData_spi_int(spi_rx_buf,&anal_cnt,CAN_POS_DIV);	
		mems.imu_att.z=floatFromData_spi_int(spi_rx_buf,&anal_cnt,CAN_POS_DIV);	
		mems.gps_pos.x=floatFromData_spi_int(spi_rx_buf,&anal_cnt,1000);	
		mems.gps_pos.y=floatFromData_spi_int(spi_rx_buf,&anal_cnt,1000);	
		mems.gps_pos.z=floatFromData_spi_int(spi_rx_buf,&anal_cnt,1000);	
		mems.Acc_CALIBRATE=charFromData_spi(spi_rx_buf,&anal_cnt);
		mems.Gyro_CALIBRATE=charFromData_spi(spi_rx_buf,&anal_cnt);
		mems.Mag_CALIBRATE=charFromData_spi(spi_rx_buf,&anal_cnt);
	}		
}

u8 SPI2_ReadWriteByte_s(u8 TxData)
{		 			 
 
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}//�ȴ���������    ʹ��if�Ļ����Ͳ��ȶ�����
	
	SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ��byte  ����
}

 
void SPI2_IRQHandler(void)//unuse now Ŀǰ����DMA1_Stream4_IRQHandler�жϽ���SPI����ͨ��  
{ 
	static char state=0,rx_cnt;
	static int spi_tx_cnt_send=0;
	char sum_r=0;
	int i,j;
	unsigned char  data_temp[8];
	char err,_cnt;
	char id;
	static u8 _data_len2 = 0,_data_cnt2 = 0;
	uint16_t data;
	static int send_flag=0;
	static float timer_sys=0;
	if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_TXE) != RESET) {
			SPI2_ReadWriteByte_s(spi_tx_buf[spi_tx_cnt_send++]); 
			if(spi_tx_cnt_send>=spi_tx_cnt&&spi_flag_pi[1]==0)//������Ͽ������¸�ֵ
			{ spi_flag_pi[1]=1;
			}
	}

	if(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) != RESET)	
	{
		if(spi_flag_pi[1]==1){//data can change
			spi_flag_pi[1]=0;
			spi_dt[0] = Get_Cycle_T(16); 
		
			timer_sys+=spi_dt[0];
			if(timer_sys>0.1){timer_sys=0;
				slave_send(3);//ϵͳ״̬
			}else{
				if(send_flag==1){
				send_flag=0;
				slave_send(1);//��̬
				}//�����������¸�ֵ
				else{
				send_flag=1;
				slave_send(2);
				}		
			}
			 spi_tx_cnt_send=0;
		}					
		
		data = SPI_I2S_ReceiveData(SPI2);//�ж϶�ȡSPI����
	
		if(state==0&&data==0xFB)
		{
			state=1;
			spi_rx_buf[0]=data;
		}
		else if(state==1&&data==0xFC)
		{
			state=2;
			spi_rx_buf[1]=data;
		}
		else if(state==2&&data>0&&data<0XF1)
		{
			state=3;
			spi_rx_buf[2]=data;
		}
		else if(state==3&&data<SPI_BUF_SIZE)
		{
			state = 4;
			spi_rx_buf[3]=data;
			_data_len2 = data;
			_data_cnt2 = 0;
		}
		else if(state==4&&_data_len2>0)
		{
			_data_len2--;
			spi_rx_buf[4+_data_cnt2++]=data;
			if(_data_len2==0)
				state= 5;
		}
		else if(state==5)
		{
			state = 0;
			spi_rx_buf[4+_data_cnt2]=data;
			spi_rx_cnt=4;
			slave_rx(spi_rx_buf,_data_cnt2+5);
			spi_rx_cnt_all++;
		}
		else
			state = 0;
		
		//ͬ������
		if(spi_tx_cnt_send>=spi_tx_cnt&&spi_flag_pi[1]==0)//������Ͽ������¸�ֵ
		{ spi_flag_pi[1]=1;
		}else
			SPI2_ReadWriteByte_s(spi_tx_buf[spi_tx_cnt_send++]); 
	}
}
 