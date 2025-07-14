/* Includes ------------------------------------------------------------------*/
#include "Custom_SPI_DEVICE.h"
#include "spi.h"
#include "wsled.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
//#define	DBG_LOG(func,num)				(printf("%s:%d\r\n",func,num))
/* Private variables ---------------------------------------------------------*/

/*SPI variable*/
unsigned char Dummy=0x5A;

unsigned char DataRxBuffer[DataSize+CheckSumSize] = {0};	
unsigned char DataTxBuffer[DataSize+CheckSumSize] = {0};	

extern unsigned char Initready_flag;

unsigned char RxDisplay_flag = 0;
unsigned char DataReady_flag = 0;
DMA_InitTypeDef DMA_InitStructure_Slave;

/* Private functions ---------------------------------------------------------*/
static void Delay(uint32_t u32Delay)
{
	//uint32_t i;
	//i = 0xffff;
	while(u32Delay--);
}

void Custom_SPI_DMABufferStart(void)
{
	NVIC_InitTypeDef 	NVIC_InitStructure;

	DMA_InitStructure_Slave.DMA_Priority = DMA_Priority_High;	//RX	
	DMA_InitStructure_Slave.DMA_DIR = DMA_DIR_PeripheralToMemory ;
	DMA_InitStructure_Slave.DMA_Memory0BaseAddr = (uint32_t)&DataRxBuffer[0];
	DMA_InitStructure_Slave.DMA_BufferSize = (uint32_t)(DataSize+CheckSumSize);
	DMA_Init(DMA1_Stream3, &DMA_InitStructure_Slave);

	DMA_InitStructure_Slave.DMA_Priority = DMA_Priority_High;	//TX
	DMA_InitStructure_Slave.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
	DMA_InitStructure_Slave.DMA_Memory0BaseAddr = (uint32_t)&DataTxBuffer[0];
	DMA_InitStructure_Slave.DMA_BufferSize = (uint32_t)(DataSize+CheckSumSize);
	DMA_Init(DMA1_Stream4, &DMA_InitStructure_Slave);
	
	DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);	
	/* I2S DMA IRQ Channel configuration */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);	
	/* I2S DMA IRQ Channel configuration */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		

	/* Enable DMA SPI RX Stream */
	DMA_Cmd(DMA1_Stream3,ENABLE);

	/* Enable DMA SPI TX Stream */
	DMA_Cmd(DMA1_Stream4,ENABLE);
	
	/* Enable SPI DMA RX Requsts */
	SPI_I2S_DMACmd(SPI_DEVICE, SPI_I2S_DMAReq_Rx, ENABLE);
	/* Enable SPI DMA TX Requsts */
	SPI_I2S_DMACmd(SPI_DEVICE, SPI_I2S_DMAReq_Tx, ENABLE);
	
	/* The Data transfer is performed in the SPI using Direct Memory Access */
	SPI_Cmd(SPI_DEVICE, ENABLE); /*!< SPI_DEVICE_SPI enable */

}

void Custom_SPI_DMABufferWait(void)
{
	/* Waiting the end of Data transfer */
	/* Clear DMA Transfer Complete Flags */
	DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);
	DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);
	
	/* Disable DMA SPI RX Stream */
	DMA_Cmd(DMA1_Stream3,DISABLE);
	/* Disable DMA SPI TX Stream */
	DMA_Cmd(DMA1_Stream4,DISABLE);
	
	/* Disable SPI DMA RX Requsts */
	SPI_I2S_DMACmd(SPI_DEVICE, SPI_I2S_DMAReq_Rx, DISABLE);
	/* Disable SPI DMA TX Requsts */
	SPI_I2S_DMACmd(SPI_DEVICE, SPI_I2S_DMAReq_Tx, DISABLE);
	
	SPI_Cmd(SPI_DEVICE, DISABLE);
}

void Custom_SPI_DMABufferConfig(void)
{

	/* DMA configuration -------------------------------------------------------*/
	/* Deinitialize DMA Streams */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);

	DMA_DeInit(DMA1_Stream3);//RX
	DMA_DeInit(DMA1_Stream4);//TX
	
	/* Configure DMA Initialization Structure */
//	DMA_InitStructure.DMA_BufferSize = DataSize;
	DMA_InitStructure_Slave.DMA_FIFOMode = DMA_FIFOMode_Disable ;
	DMA_InitStructure_Slave.DMA_FIFOThreshold = DMA_FIFOThreshold_Full ;
	DMA_InitStructure_Slave.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
	DMA_InitStructure_Slave.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure_Slave.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure_Slave.DMA_Mode = DMA_Mode_Normal;

	DMA_InitStructure_Slave.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure_Slave.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure_Slave.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	/* Configure RX DMA */
	DMA_InitStructure_Slave.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI_DEVICE->DR)) ;
	DMA_InitStructure_Slave.DMA_Channel = DMA_Channel_0 ;
}


void Custom_SPI_DEVICE_TestCommand(void)
{
	uint16_t i = 0;
	#if 1
		if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3) == RESET)//rx
		{
			#if 1	//debug
			Custom_SPI_DMABufferWait(); 

			if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)==RESET)
			{ 
				for(i=0;i<DataSize;i++)
					DataTxBuffer[i]=i;

				Custom_SPI_DMABufferStart();
			}	
			#endif		
		}

	#endif
}

void Custom_SPI_DEVICE_Slave_Config(void)
{
//	GPIO_InitTypeDef 	GPIO_InitStructure;
	SPI_InitTypeDef  	SPI_InitStructure;
//	EXTI_InitTypeDef   	EXTI_InitStructure;
//	NVIC_InitTypeDef   NVIC_InitStructure;

	/* Enable the SPI periph */
	SPI_DEVICE_CLK_INIT(SPI_DEVICE_CLK, ENABLE);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI_DEVICE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;	
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;//SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SLAVE_SPI_BAUDRATE;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_DEVICE, &SPI_InitStructure);

	/* Enable SPI DMA RX Requsts */
	SPI_I2S_DMACmd(SPI_DEVICE, SPI_I2S_DMAReq_Rx, ENABLE);
	/* Enable SPI DMA TX Requsts */
	SPI_I2S_DMACmd(SPI_DEVICE, SPI_I2S_DMAReq_Tx, ENABLE);	

}

void Custom_SPI_DEVICE_Slave_EXTI_Config(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	SPI_InitTypeDef  	SPI_InitStructure;
	EXTI_InitTypeDef   	EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* Enable the SPI periph */
	SPI_DEVICE_CLK_INIT(SPI_DEVICE_CLK, ENABLE);

	/* Enable SCK, MOSI, MISO and NSS GPIO clocks */
	RCC_AHB1PeriphClockCmd(SPI_DEVICE_MOSI_GPIO_CLK|SPI_DEVICE_MISO_GPIO_CLK |SPI_DEVICE_SCK_GPIO_CLK , ENABLE);

	GPIO_PinAFConfig(SPI_DEVICE_SCK_GPIO_PORT, SPI_DEVICE_SCK_GPIO_SOURCE, SPI_DEVICE_SCK_GPIO_AF);			//CLK
	GPIO_PinAFConfig(SPI_DEVICE_MISO_GPIO_PORT, SPI_DEVICE_MISO_GPIO_SOURCE, SPI_DEVICE_MISO_GPIO_AF);   	//MISO
	GPIO_PinAFConfig(SPI_DEVICE_MOSI_GPIO_PORT, SPI_DEVICE_MOSI_GPIO_SOURCE, SPI_DEVICE_MOSI_GPIO_AF);		//MOSI

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	/*!< Configure SPI_DEVICE_SPI pins: SCK */
	GPIO_InitStructure.GPIO_Pin = SPI_DEVICE_SCK_PIN;
	GPIO_Init(SPI_DEVICE_SCK_GPIO_PORT, &GPIO_InitStructure);
	/*!< Configure SPI_DEVICE_SPI pins: MISO */
	GPIO_InitStructure.GPIO_Pin =  SPI_DEVICE_MISO_PIN;
	GPIO_Init(SPI_DEVICE_MISO_GPIO_PORT, &GPIO_InitStructure);  
	/*!< Configure SPI_DEVICE_SPI pins: MOSI */
	GPIO_InitStructure.GPIO_Pin =  SPI_DEVICE_MOSI_PIN;
	GPIO_Init(SPI_DEVICE_MOSI_GPIO_PORT, &GPIO_InitStructure);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI_DEVICE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;	
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SLAVE_SPI_BAUDRATE;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_DEVICE, &SPI_InitStructure);
	/* Enable SPI DMA RX Requsts */
	SPI_I2S_DMACmd(SPI_DEVICE, SPI_I2S_DMAReq_Rx, ENABLE);
	/* Enable SPI DMA TX Requsts */
	SPI_I2S_DMACmd(SPI_DEVICE, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_Cmd(SPI_DEVICE, ENABLE); //Ê¹ÄÜSPIÍâÉè
}   

unsigned char Get_CheckSum(unsigned char data[],uint32_t len)
{
	int i,res=0;
	for(i=0;i<len;i++)
	{
		res += data[i];
	}
	return (unsigned char)(0xFF - res);
}


void EXTI15_10_IRQHandler(void)
{
	unsigned char i = 0;
	static uint16_t counter = 1;
	
	if(EXTI_GetITStatus(SPI_DEVICE_CS_EXTI_Line) != RESET)
	{
		if (Is_SPI_NCS_LOW())	//ready to receive data
		{
 
			for (i=0;i<DataSize;i++)
			{
				DataTxBuffer[i]= i+0x70*counter;
			}
			DataTxBuffer[DataSize+CheckSumSize-1] = Get_CheckSum(DataTxBuffer,DataSize);

			counter++;
 
			Custom_SPI_DMABufferStart();			
		}

		EXTI_ClearITPendingBit(SPI_DEVICE_CS_EXTI_Line);
	}
}

void DMA1_Stream3_IRQHandler(void)//rx
{
	static char state_spi=0,rx_cnt;
	static int spi_tx_cnt_send=0;
	char sum_r=0;
	int i,j;
	unsigned char  data_temp[8];
	char err,_cnt;
	char id;
	static u8 _data_len2_spi = 0,_data_cnt2_spi = 0;
	unsigned char data_spi;
	static int send_flag=0;
	static float timer_sys=0;
	/* Test on DMA Stream Transfer Complete interrupt */
	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3))
	{
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
		DMA_ITConfig(DMA1_Stream3, DMA_IT_TCIF3, DISABLE);
		if(!wsled.led_lock){
			for (i=0;i<(DataSize+CheckSumSize);i++)
			{

				data_spi = DataRxBuffer[i];//ÖÐ¶Ï¶ÁÈ¡SPIÊý¾Ý
			
				if(state_spi==0&&data_spi==0xFE)
				{
					state_spi=1;
					spi_rx_buf[0]=data_spi;
				}
				else if(state_spi==1&&data_spi==0xFC)
				{
					state_spi=2;
					spi_rx_buf[1]=data_spi;
				}
				else if(state_spi==2&&data_spi>0&&data_spi<0XF1)
				{
					state_spi=3;
					spi_rx_buf[2]=data_spi;
				}
				else if(state_spi==3&&data_spi<SPI_BUF_SIZE)
				{
					state_spi = 4;
					spi_rx_buf[3]=data_spi;
					_data_len2_spi = data_spi;
					_data_cnt2_spi = 0;
				}
				else if(state_spi==4&&_data_len2_spi>0)
				{
					_data_len2_spi--;
					spi_rx_buf[4+_data_cnt2_spi++]=data_spi;
					if(_data_len2_spi==0)
						state_spi= 5;
				}
				else if(state_spi==5)
				{
					state_spi = 0;
					spi_rx_buf[4+_data_cnt2_spi]=data_spi;
					spi_rx_cnt=4;
					slave_rx(spi_rx_buf,_data_cnt2_spi+5);
					spi_rx_cnt_all++;
				}
				else
					state_spi = 0;
			}	
		}
		Custom_SPI_DMABufferWait(); 
    Custom_SPI_DMABufferStart();
	}
}

void DMA1_Stream4_IRQHandler(void)//tx
{	
	static char state_spi=0,rx_cnt;
	static int spi_tx_cnt_send=0;
	char sum_r=0;
	int i,j;
	unsigned char  data_temp[8];
	char err,_cnt;
	char id;
	static u8 _data_len2_spi = 0,_data_cnt2_spi = 0;
	unsigned char data_spi;
	static int send_flag=0;
	static float timer_sys=0;
	/* Test on DMA Stream Transfer Complete interrupt */
	if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4))
	{
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
		DMA_ITConfig(DMA1_Stream4, DMA_IT_TCIF4, DISABLE);

	//
		if(!wsled.led_lock){//spi_flag_pi[1]==1||1){//data can change
			spi_flag_pi[1]=0;
			spi_dt[0] = Get_Cycle_T(16); 

			timer_sys+=spi_dt[0];
			
  #if MCU_TINYPALE
			slave_send(30);//simple car
	#else
		#if USE_WHEEL
			slave_send(25);//21 6Í¨µÀÒ£¿ØÆ÷  24 ÔÆ×¿H12Ò£¿ØÆ÷	 ÂÖ×ã	
		#else
			  if(timer_sys>0.02){//50Hz
					timer_sys=0;
					slave_send(36);
				}
				else
					slave_send(26);//21 6Í¨µÀÒ£¿ØÆ÷  24 ÔÆ×¿H12Ò£¿ØÆ÷
		#endif
	#endif
			spi_tx_cnt_send=0;
		
			for(i=0;i<DataSize;i++)
				DataTxBuffer[i]=spi_tx_buf[i];
		}		
	}
}

