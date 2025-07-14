#include "bat.h"
#include "include.h"		 
#include "gait_math.h"
//��ʼ��ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOAʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��

  //�ȳ�ʼ��ADC1ͨ��5 IO��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//PA5 ͨ��5;//PA5 ͨ��5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��  
#if defined(LEG_USE_AD)
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;//PA5 ͨ��5;//PA5 ͨ��5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��  
#endif
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	 
 	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
	
 
	ADC_Cmd(ADC1, ENABLE);//����ADת����	

}				  
#define ADC1_DR_Address    ((uint32_t)0x4001204C) 
uint16_t ADC_Value[5]={0};//AD??

void ADC_Configuration(void)
{

 ADC_InitTypeDef       ADC_InitStructure; 
  ADC_CommonInitTypeDef ADC_CommonInitStructure; 
  DMA_InitTypeDef       DMA_InitStructure; 
  GPIO_InitTypeDef      GPIO_InitStructure; 

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 

 DMA_DeInit(DMA2_Stream0); 
  
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;   
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address; 
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value; 
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; 
  DMA_InitStructure.DMA_BufferSize = 5; 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
  DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;          
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
  DMA_Init(DMA2_Stream0, &DMA_InitStructure); 
  DMA_Cmd(DMA2_Stream0, ENABLE); 


  /* Configure ADC1 Channel6 pin as analog input ******************************/ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; 
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  /* ADC Common Init **********************************************************/ 
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; 
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; 
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; 
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; 
  ADC_CommonInit(&ADC_CommonInitStructure); 

  /* ADC1 Init ****************************************************************/ 
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ScanConvMode = ENABLE; 
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None ; 
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
  ADC_InitStructure.ADC_NbrOfConversion =5; 
  ADC_Init(ADC1, &ADC_InitStructure); 

/* ADC1 regular channe6 configuration *************************************/ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_3Cycles); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_3Cycles); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_3Cycles); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_3Cycles); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_15Cycles); 
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); 
	ADC_DMACmd(ADC1, ENABLE); 
	ADC_Cmd(ADC1, ENABLE); 
  ADC_SoftwareStartConv(ADC1);	
}
//���ADCֵ
//ch: @ref ADC_channels 
//ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_Channel_0~ADC_Channel_16
//����ֵ:ת�����
#if USE_VER_6
float k_ad=0.00875;
#else
float k_ad=12.38/1370;
#endif
float Get_Adc(u8 ch)   
{ 
	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_112Cycles );	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��			    
  
	ADC_SoftwareStartConv(ADC1);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1)*k_ad;	//�������һ��ADC1�������ת�����
}

float press_leg_end[5];
ESO_X flt_ground[5];
char ground_v[5],ground_vad[5];
float ground_press_max[5]={0,  0.1,  0.1,  4, 7.6};
float ground_press_min[5]={0,  0.1,  0.1,  2.4, 1.9};
float k_check_force=0.28;
float k_check_dforce=0.23;
void get_leg_press(float dt)
{   static float press_leg_endr[5];
    char i;
	  static char init;
	  if(!init){init=1;flt_ground[1].r0=420;}
		DigitalLPF(Get_Adc(0), &press_leg_end[1], 66, dt);
		DigitalLPF(Get_Adc(1), &press_leg_end[2], 66, dt);
		DigitalLPF(Get_Adc(2), &press_leg_end[3], 66, dt);
		DigitalLPF(Get_Adc(3), &press_leg_end[4], 66, dt);
//		press_leg_end[1]=Get_Adc(1);
//		press_leg_end[2]=Get_Adc(4);
//		press_leg_end[3]=Get_Adc(5);
//		press_leg_end[4]=Get_Adc(6);
		
		for(i=0;i<4;i++)
			vmc[i].ground_force[0]=flt_ground[i+1].v1;
		
	  for(i=2;i<5;i++){
			flt_ground[i].h0=flt_ground[1].h0=dt;
			flt_ground[i].r0=flt_ground[1].r0;
	  }
	  for(i=1;i<5;i++){
			OLDX_SMOOTH_IN_ESOX(&flt_ground[i],
			LIMIT(press_leg_end[i]-vmc_all.param.ground_force[i-1][0],0,ABS(vmc_all.param.ground_force[i-1][1]-vmc_all.param.ground_force[i-1][0]))
			/ABS(vmc_all.param.ground_force[i-1][1]-vmc_all.param.ground_force[i-1][0]));
			vmc[i-1].ground_force[1]=flt_ground[i].v2/10;
			if((vmc[i-1].ground_force[1]>k_check_dforce&&0)||vmc[i-1].ground_force[0]>k_check_force)
				ground_vad[i]=1;
			else
				ground_vad[i]=0;
		}
		
		press_leg_endr[1]=press_leg_end[1];
		press_leg_endr[2]=press_leg_end[2];
		press_leg_endr[3]=press_leg_end[3];
		press_leg_endr[4]=press_leg_end[4];
}	

//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		Delay_ms(5);
	}
	return temp_val/times;
} 
	 
BAT bat;
void Bat_protect(float dt)
{
static u8 state;
static u16 cnt[5];	
static  float temp,temp1;
	#if defined(LEG_USE_AD)
	 get_leg_press(dt);
	#endif
	switch(state)
	{
		case 0:
			  if(Get_Adc(4)!=0){
			  temp+=Get_Adc(4);
				cnt[0]++;	
				}
				if(cnt[0]>20){
					state=1;
					temp1=temp/cnt[0];
					if(temp1>11.1-1&&temp1<12.6+1)
					  bat.bat_s=3;
					else if(temp1>14.8-1&&temp1<16.4+1)
						bat.bat_s=4;
				  else if(temp1>3.7*2-1&&temp1<4.2*2+1)
						bat.bat_s=2;
					else
						bat.bat_s=2;
					bat.full=4.25*bat.bat_s;
					temp=0;
				}	  
		break;
	  case 1:
			 if(Get_Adc(4)!=0){
			  bat.origin=Get_Adc(4);
				temp+=bat.origin;
				cnt[1]++;	
				}
				if(cnt[1]>2/dt){		
					bat.average=temp/cnt[1];cnt[1]=0;
					bat.percent=LIMIT((bat.average-bat.bat_s*3.12)/(bat.full-bat.bat_s*3.12),0,0.99);

          temp=0;		
          switch(bat.bat_s){
						case 2:
						 	if(bat.percent<0.03)//(bat.average<11.1-0.6||bat.average<11.1-0.6+(12.6-11.1)*bat.protect_percent)
							bat.low_bat=1;	
							else			
							bat.low_bat=0;
						break;
            case 3:
						 	if(bat.percent<0.03)//(bat.average<11.1-0.6||bat.average<11.1-0.6+(12.6-11.1)*bat.protect_percent)
							bat.low_bat=1;	
							else			
							bat.low_bat=0;
						break;
						case 4:
						if(bat.percent<0.6)//bat.average<14.8-0.6||bat.average<14.8-0.6+(16.4-14.8)*bat.protect_percent)
							bat.low_bat_cnt++;
						else
							bat.low_bat_cnt=0;
						 if(bat.low_bat_cnt>2/0.05)
						  bat.low_bat=1;	
							else			
							bat.low_bat=0;
						break;
 					}
				}	  
		break;
	}
}