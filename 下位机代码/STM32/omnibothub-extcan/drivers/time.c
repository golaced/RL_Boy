

#include "time.h"
#include "include.h"
#include "scheduler.h"
#include "init.h"
#define SYS_TIMx					TIM2
#define SYS_RCC_TIMx			RCC_APB1Periph_TIM2

void TIM_CONF()   //APB1  84M
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		/* ???? */
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx,ENABLE);

		TIM_DeInit(SYS_TIMx);
	
	/* ????????????(???) */
    TIM_TimeBaseStructure.TIM_Period=1000;
	
    /* ?? TIM_Period?????????????? */
	  /* ???????72 */
    TIM_TimeBaseStructure.TIM_Prescaler= 84 - 1;
	
		/* ??????????????,?????? */
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;   //????
	
		TIM_TimeBaseInit(SYS_TIMx,&TIM_TimeBaseStructure);

		TIM_ClearFlag(SYS_TIMx,TIM_FLAG_Update);

		TIM_ITConfig(SYS_TIMx,TIM_IT_Update,ENABLE);
		
		
    TIM_Cmd(SYS_TIMx, ENABLE);																		
    
    RCC_APB1PeriphClockCmd(SYS_RCC_TIMx , DISABLE);		/*???????*/  
}
void TIM_NVIC()
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TIME_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_TIME_S;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM_INIT()
{
    TIM_CONF();
    TIM_NVIC();
	
		/* TIM2 ?????,???? */
	  RCC_APB1PeriphClockCmd(SYS_RCC_TIMx , ENABLE);
}


volatile uint32_t sysTickUptime = 0;

#define TICK_PER_SECOND 2000 
#define TICK_US	(1000000/TICK_PER_SECOND)

void  SysTick_Configuration(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	uint32_t         cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
	cnts = cnts / 8;

	SysTick_Config(cnts);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

uint32_t GetSysTime_us(void) 
{
	register uint32_t ms;
	u32 value;
	ms = sysTickUptime;
	value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
	return value;
}

void Delay_us(uint32_t us)
{
    uint32_t now = GetSysTime_us();
    while (GetSysTime_us() - now < us);
}

void Delay_ms(uint32_t ms)
{
    while (ms--)
        Delay_us(1000);
}

uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = SYS_TIMx->CNT;
 	return temp;
}

int time_1h,time_1m,time_1s,time_1ms;

void sys_time()
{ 

  if(time_1ms < 999)
	{
    time_1ms++;

		Loop_check();
	}
	else
	{
		
    time_1ms =0;
	  if(time_1s<59)
	  {
      time_1s++;
			loop.err_flag_1s=loop.err_flag;
			
			loop.err_flag_rate=loop.err_flag_1s/1000.;
			loop.err_flag=0;
		}
		else
		{
			time_1s = 0;
			if(time_1m<59)
			{
				time_1m++;
			}
			else
			{
				time_1m = 0;
				if(time_1h<23)
				{
					time_1h++;
				}
				else
				{
					time_1h = 0;
				}
			}
		}
	}
}
volatile float Cycle_T[GET_TIME_NUM][4];

enum
{
	NOW = 0,
	OLD,
	NEW,
	DT_LAST
};

float Get_Cycle_T(u8 item)	
{
	Cycle_T[item][OLD] = Cycle_T[item][NOW];	//????¦Å????
	Cycle_T[item][NOW] = (float)GetSysTime_us()/1000000.0f; //GetSysTime_us()/1000000.0f; //???¦Å????
	if(Cycle_T[item][NOW]>Cycle_T[item][OLD]){
		Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//?????????????
		Cycle_T[item][DT_LAST] = Cycle_T[item][NEW];
  }
	else
		Cycle_T[item][NEW] = Cycle_T[item][DT_LAST];

	if(Cycle_T[item][NEW]>0.5)
		return 0.5;
	else
		return Cycle_T[item][NEW];
}


void Cycle_Time_Init()
{
	u8 i;
	for(i=0;i<GET_TIME_NUM;i++)
	{
		Get_Cycle_T(i);
	}

}


void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///???TIM3???
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//?????????
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //????????
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //?????????
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//?????TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //?????????3?????§Ø?
	TIM_Cmd(TIM3,ENABLE); //???????3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //?????3?§Ø?
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2; //????????1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3; //???????3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}





