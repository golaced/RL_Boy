#include   "beep.h"
#include   "time.h"
#include   "bat.h"
#include   "include.h"
#include 	 "led_fc.h"
#include 	 "can.h"
u8 music_sel;
//  11  21
//音阶(0~3)音高(0~7)   ||   STOP循环次数(0~7) 休止符长度(0~7)
#define B1 13
#define B2 16
#define B3 15
u8 start_music_px4[]={
 B1,12, B2,12, B3,12,  B1,12, B2,12, B3,12,  B1,12, B2,12, B3,12,   B1,12, B2,12, B3,12,  B2,12,  B3,11,  B2,11, B3,11,  B2,11, B3,43,
};

u8 start_music_pi[]={
  27,01, 00,01,   00,59 , 00,59
};

u8 start_music_pi_thread[]={
  27,01, 00,01,   17,01, 00,01,  00,59 , 00,59
};

u8 start_music_windows[]={
 21,21, 01,11, 21,21, 01,11, 21,21,01,11,      11,13,  15,13,    13,11, 01,11, 13,11, 01,11, 13,11, 01,11, 
	21,23,   15,41, 01,11, 15,31, 01,11, 15,21, 01,11,  15,11, 01,11,
};

u8 start_music_micro[]={
 21,21, 01,11, 21,21, 01,11, 21,21,01,11,      11,13,  15,13,    13,11, 01,11, 13,11, 01,11, 13,11, 01,11, 
	21,23,   15,41, 01,11, 15,31, 01,11, 15,21, 01,11,  15,11, 01,11,
};

u8 mems_gps_music[]={
 15,44,  17,33, 17,33
};


u8 mems_right_music[]={
 05,64,  00,13, 12,64
};

u8 mems_error_music[]={
 02,34, 00,32, 02,34, 00,32, 
};

u8 mission_music[]={
 22,35, 22,35, 22,65,  
};

u8 mems_wayup_music[]={
 15,52,  17,32, 17,32,
};

u8 bat_error_music[]={
 07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  
 07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32,  07,33, 00,32, 07,33, 00,32, 	
};

u8 rc_error_music[]={
 27,11, 00,11,   27,11, 00,11,  27,11, 00,11, 00,39 , 00,39
};

u8 hml_cal_music[]={
 13,11, 00,11,   13,11, 00,11,  13,11, 00,11, 00,39 , 00,39
};


u8 bldc_cal_zero_music[]={
 05,64,  00,13, 12,64
};

u8 bldc_cal_init_music[]={
  03,64,  00,13, 15,34
};

u8 bldc_reset_err_music[]={
 02,34, 00,32, 02,34, 00,32, 
};

u8 gait_switch_music[]={
  15,14,  00,13, 12,14
};

u8 start_music_pix[]={
 04,32, 24,22, 05,32, 
 04,32, 24,22, 05,32,
 04,32, 24,22, 05,32,
};

u8 dj_cal_music1[]={
 27,01, 00,01,   00,39 , 00,39
};
u8 dj_cal_music2[]={
 27,01, 00,01,   27,01, 00,01,	00,39 , 00,39
};
u8 dj_cal_music3[]={
 27,01, 00,01,   27,01, 00,01, 	27,01, 00,01,	00,39 , 00,39
};
u8 dj_cal_music4[]={
 27,01, 00,01,   27,01, 00,01,	27,01, 00,01,  27,01, 00,01,	00,39 , 00,39
};
u8 dj_cal_music5[]={
 27,01, 00,01,   27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	00,39 , 00,39
};
u8 dj_cal_music6[]={
 27,01, 00,01,   27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	00,39 , 00,39
};
u8 dj_cal_music7[]={
 27,01, 00,01,   27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	00,39 , 00,39
};
u8 dj_cal_music8[]={
 27,01, 00,01,   27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	27,01, 00,01,	00,39 , 00,39
};

u8 fc_state_beep[3];
u8 beep_state[]={
 00,11, 00,21,   00,11, 00,21,  00,11, 00,21, 00,11, 00,11,
};

u8 bldc_state_beep[4];
u8 beep_bldc_state[]={
 00,11, 00,22,   00,11, 00,22,  00,11, 00,22,   00,11, 00,22, 00,22, 00,22,
};


u8 fc_save_gps_beep;
u8 beep_gps_save[]={
 00,11, 00,21,   00,11, 00,21,  
};

u8 test_to24;
u16 Beat_delay[7]={0,62,94,125,125,187,250};
void Play_Music(u8 *music, u16 st,u16 ed)
{
	static u8 flag;
	u16 i;
  for(i=st;i<ed-st;i++)
   {
		 u8 level=music[i*2]/10;
		 u8 tone=music[i*2]%10;
		 Tone(level,tone);
		 u8 loop=music[i*2+1]%10;
		 u8 beat=music[i*2+1]/10;
     Delay_ms(Beat_delay[beat]*loop);	 
		 if(music_sel==START_BEEP){
		   flag=!flag;
		 }
	 }	
   Tone(0,0);		 
}

char beep_state_task;
u16 beep_state_task_cnt;

void Reset_Beep_Task(void)
{
	Tone(0,0);
	beep_state_task=0;
  beep_state_task_cnt=0;
}

u8 Play_Music_In_Task(u8 *music, u16 st,u16 ed,u8 en,float dt)
{ static u8 state=0;
	static u16 i,cnt;
	
	switch(beep_state_task)
	{
		case 0:
	     if(en)
	     {beep_state_task=1;i=st;beep_state_task_cnt=0;}
	  break;
	  case 1:
			 if(en) 
			 {
				u8 level=music[i*2]/10;
				u8 tone=music[i*2]%10;
				u8 loop=music[i*2+1]%10;
				u8 beat=music[i*2+1]/10;
				 
				 if(beep_state_task_cnt++>Beat_delay[beat]*loop/1000./dt){Tone(level,tone);i++;beep_state_task_cnt=0;} 
			 }
	     else
						{Tone(0,0);beep_state_task=0;  }
			 if(i>ed)
			 {beep_state_task=0;Tone(0,0);	return 1;}
		break;
	} 
	return 0;
}

void Play_Music_Direct(u8 sel)
{
	music_sel=0;
  switch(sel)
	{
		case MEMS_RIGHT_BEEP:
	  Play_Music(mems_right_music,0,sizeof(mems_right_music)/2);	
	  break;
		case START_BEEP:
		music_sel=	START_BEEP;
		Play_Music(start_music_px4,0,sizeof(start_music_px4)/2);	
		break;
		case MEMS_ERROR_BEEP:
		Play_Music(mems_error_music,0,sizeof(mems_error_music)/2);		
		break;
		case MEMS_WAY_UPDATE:
		Play_Music(mems_wayup_music,0,sizeof(mems_wayup_music)/2);		
		break;
		case MEMS_GPS_RIGHT:
		Play_Music(mems_gps_music,0,sizeof(mems_gps_music)/2);		
		break;
	}
}

char Play_Music_Task(u8 sel,float dt)//<-----------------
{
	char finish=0;
  switch(sel)
	{
		case MEMS_RIGHT_BEEP:
	  finish=Play_Music_In_Task(mems_right_music,0,sizeof(mems_right_music)/2,1,dt);	
	  break;
		case START_BEEP:
		finish=Play_Music_In_Task(start_music_windows,0,sizeof(start_music_windows)/2,1,dt);	
		break;
		case BEEP_BLDC_SPI_CONNECT:
		finish=Play_Music_In_Task(start_music_pi,0,sizeof(start_music_pi)/2,1,dt);	
		break;
		case BEEP_BLDC_SPI_CONNECT_THREAD_UP:
		finish=Play_Music_In_Task(start_music_pi_thread,0,sizeof(start_music_pi_thread)/2,1,dt);	
		break;		
		case MEMS_GPS_RIGHT:
		finish=Play_Music_In_Task(mems_gps_music,0,sizeof(mems_gps_music)/2,1,dt);		
		break;
		case MEMS_ERROR_BEEP:
		finish=Play_Music_In_Task(mems_error_music,0,sizeof(mems_error_music)/2,1,dt);		
		break;
		case BEEP_MISSION:
		finish=Play_Music_In_Task(mission_music,0,sizeof(mission_music)/2,1,dt);			
		break;
		case BAT_ERO_BEEP:
		#if defined(DEBUG_MODE)
    Tone(0,0);
		#else
		finish=Play_Music_In_Task(bat_error_music,0,sizeof(bat_error_music)/2,1,dt);		
		#endif
		break;
		case RC_ERO_BEEP:
		;//Play_Music_In_Task(rc_error_music,0,sizeof(rc_error_music)/2,1,dt);		
		break;
		case BEEP_STATE:
		beep_state[0]=fc_state_beep[0];	
		beep_state[2]=fc_state_beep[1];	
		beep_state[4]=fc_state_beep[2];	
		#if defined(DEBUG_MODE)
    Tone(0,0);
		#else
		finish=Play_Music_In_Task(beep_state,0,sizeof(beep_state)/2,1,dt);		
		#endif
		break;
		case BEEP_GPS_SAVE:
		beep_gps_save[0]=beep_gps_save[2]=fc_save_gps_beep;	
		finish=Play_Music_In_Task(beep_gps_save,0,sizeof(beep_gps_save)/2,1,dt);		
		break;
		case BEEP_HML_CAL:
		finish=Play_Music_In_Task(hml_cal_music,0,sizeof(hml_cal_music)/2,1,dt);		
		break;
		case BEEP_DJ_CAL1:
			switch(dj_sel){
				case 0: finish=Play_Music_In_Task(dj_cal_music1,0,sizeof(dj_cal_music1)/2,1,dt);break;		
				case 1: finish=Play_Music_In_Task(dj_cal_music2,0,sizeof(dj_cal_music2)/2,1,dt);break;	
				case 2: finish=Play_Music_In_Task(dj_cal_music3,0,sizeof(dj_cal_music3)/2,1,dt);break;	
				case 3: finish=Play_Music_In_Task(dj_cal_music4,0,sizeof(dj_cal_music4)/2,1,dt);break;	
				case 4: finish=Play_Music_In_Task(dj_cal_music5,0,sizeof(dj_cal_music5)/2,1,dt);break;	
				case 5: finish=Play_Music_In_Task(dj_cal_music6,0,sizeof(dj_cal_music6)/2,1,dt);break;	
				case 6: finish=Play_Music_In_Task(dj_cal_music7,0,sizeof(dj_cal_music7)/2,1,dt);break;	
				case 7: finish=Play_Music_In_Task(dj_cal_music8,0,sizeof(dj_cal_music8)/2,1,dt);break;	
			}
			
		break;
		case BEEP_BLDC_ZERO_CAL:    finish=Play_Music_In_Task(bldc_cal_zero_music,0,sizeof(bldc_cal_zero_music)/2,1,dt);	break;
		case BEEP_BLDC_ZERO_INIT:   finish=Play_Music_In_Task(bldc_cal_init_music,0,sizeof(bldc_cal_init_music)/2,1,dt);	break;
		case BEEP_BLDC_GAIT_SWITCH: finish=Play_Music_In_Task(gait_switch_music,0,sizeof(gait_switch_music)/2,1,dt);	break;
		case BEEP_BLDC_RESET_ERR:   finish=Play_Music_In_Task(bldc_reset_err_music,0,sizeof(bldc_reset_err_music)/2,1,dt);	break;
		case BEEP_BLDC_STATE:
			
    #if defined(DEBUG_MODE)
    Tone(0,0);
		#else
		finish=Play_Music_In_Task(beep_bldc_state,0,sizeof(beep_bldc_state)/2,1,dt);		
		#endif
		break;
			
		default:
			Tone(0,0);
		break;
	}
}

void Beep_Init(u32 arr,u32 psc)
{		 					 

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4); //GPIOF9复用为定时器14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = arr/2;
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM14
	
	delay_ms(100);
	Play_Music_Direct(START_BEEP);
	delay_ms(100);
  Tone(0,0);
}  

u16 tone_table[3][8]={
     {0,261,293,329,349,391,440,493},
     {0,523,587,659,698,783,880,987},
     {0,1046,1174,1318,1396,1567,1760,1975}};
void Tone(u8 level, u8 tone)
{
	#if defined(EN_BEEP)
	u32 psc=84/2-1;
  u32 arr=1000000/tone_table[level][tone]-1;
  if(tone==0)
		arr=1000000/1-1;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = arr/2;
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM14
	#endif
} 
