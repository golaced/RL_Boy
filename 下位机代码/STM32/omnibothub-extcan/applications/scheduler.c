 #include "scheduler.h"
#include "include.h"
#include "EKF_AHRS.h"
#include "bat.h"
#include "imu.h"
#include "flash.h"
#include "led_fc.h"
#include "sbus.h"
#include "rc_mine.h"
#include "dog.h"
#include "usart_fc.h"
#include "pwm_out.h"
#include "beep.h"
#include "nav.h"
#include "ms5611.h"
#include "eso.h"
#include "mavl.h"
#include "spi.h"
#include "gps.h"
#include "usbd_cdc_vcp.h" 
#include "spl06.h" 
#include "can.h" 
#include "beep.h" 
#include "gait_math.h" 
#include "locomotion_header.h"
#include "wsled.h"
#include "Custom_SPI_Device.h"

VMC vmc[4];
VMC_ALL vmc_all;
robotTypeDef robotwb;
_OCU ocu,ocu_rx;	
POS_FORCE_PARM pos_force_p;
VMC_ROBOT_PARM vmc_robot_p;
char stand_force_enable_flag[5]={0};
float MIN_Z=-0.1;
float MAX_Z=-0.19;
float MIN_Y=-0.1;
float MAX_Y=-0.1;
float MIN_X=-0.15;
float MAX_X=0.15;
float MAX_SPD=0;
float MAX_SPD_RAD=50;
_SDK sdk;

_SYSTEM_DT system_dt;
s16 loop_cnt;
loop_t loop;
float leg_dt[GET_TIME_NUM];
float trig_test_dt[3]={0};
void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;
	loop.cnt_1s++;
	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
}

void Duty_Servo()//1ms
{
	system_dt.can_task= leg_dt[0] = Get_Cycle_T(0); 
	if(leg_dt[0]>0.00225)
		can_rx_over[4]++;
	
  CAN_motor_sm(leg_dt[0]);	
}

 
#define EN_GYRO_Z_F_ODOM 1
u8 en_nav=1;
float FLT_ACC=10;
float FLT_ATT_RT=0;//20;//15;//1.68*2;
char att_fusion_use[2]={1,1};
Vect3 vect_n_test,vect_b_test;
float FLT_ATT_RATE=0;//WS
void subscribe_imu_to_webot(robotTypeDef* rob,float dt)
{
	char i,j;
	robotwb.IMU_now.pitch=vmc_all.att[PITr];
	robotwb.IMU_now.roll=vmc_all.att[ROLr];
	robotwb.IMU_now.yaw=vmc_all.att[YAWr];
	
  for(i=0;i<3;i++)
  {
    for(j=0;j<3;j++){
    robotwb.Rb_n[i][j]=vmc_all.Rb_n[i][j];
    robotwb.Rn_b[i][j]=vmc_all.Rn_b[i][j];
		robotwb.Rb_n_noroll[i][j]=vmc_all.Rb_n_noroll[i][j];
    robotwb.Rn_b_noroll[i][j]=vmc_all.Rn_b_noroll[i][j];
    }
  }

	DigitalLPF_Double(vmc_all.att_rate[PITr], &robotwb.IMU_dot.pitch, FLT_ATT_RATE, dt);
	DigitalLPF_Double(vmc_all.att_rate[ROLr], &robotwb.IMU_dot.roll, FLT_ATT_RATE, dt);
	DigitalLPF_Double(vmc_all.att_rate[YAWr], &robotwb.IMU_dot.yaw, FLT_ATT_RATE, dt);

	robotwb.now_att=robotwb.IMU_now;
	robotwb.now_rate=robotwb.IMU_dot;
}
 

u8 UART_UP_LOAD_SEL=16;//<------------------------------UART UPLOAD DATA SEL
float ground_check_show=0;
char record_single_leg[2]={0,0};
char leg_show_sel=SINGLE_LEG_ID;
void Duty_Link()//USB 通讯 上位机
{
	static u16 cnt[3];
	static float pos_rx_timer[14];
	static float cnt_mavlink_data,cnt_rc;
	static float test_sin_t=0;
	char i;
	char cnt_curve=0;
	system_dt.link_task= leg_dt[5] = Get_Cycle_T(5); 	
	test_sin_t+=leg_dt[5]; 	
	cnt_curve=0;
 
	use_bldc_test(leg_dt[5]);//电机配置
}

void Duty_System()//遥控 保护
{  
	u8 i;	
	static u16 cnt_1,cnt_2;	
	static u8 cnt;
  static char state_ocu=0;
	static char state_sdk=0;
	float T;
	system_dt.system_task=T=leg_dt[7] = Get_Cycle_T(7); 
	
	LEDRGB_STATE(0.05);
	
	LED_SCP(io_sel_scp_scl[0]);
  LED_SCL(io_sel_scp_scl[1]);

	if(vmc_all.param.cal_flag[0]&&module.flash){
	for(i=0;i<4;i++)
		vmc_all.param.ground_force[i][0]=press_leg_end[i+1]*1.068;
		mems.Gyro_CALIBRATE=1;
	vmc_all.param.cal_flag[0]=0;
	}else if(vmc_all.param.cal_flag[1]==1&&module.flash){
		mems.Gyro_CALIBRATE=1;
		vmc_all.param.cal_flag[1]=0;
	}else if(vmc_all.param.param_save==1&&module.flash){
		WRITE_PARM();
		vmc_all.param.param_save=0;	
	}else if(vmc_all.param.cal_flag[1]==2&&module.flash){
		vmc_all.param.cal_flag[1]=0;
	}
	
	ocu.sbus_conncect=Rc_Get_SBUS.update;
	if(Rc_Get_SBUS.update)//--------------Ê¹ÓÃSBUS
	{ 
	  ocu.sbus_rc_main[0]=Rc_Get.THROTTLE=LIMIT(Rc_Get_SBUS.THROTTLE,1000,2000)	;
		ocu.sbus_rc_main[1]=Rc_Get.ROLL=my_deathzoom_rc(Rc_Get_SBUS.ROLL,50)	;
		ocu.sbus_rc_main[2]=Rc_Get.PITCH=my_deathzoom_rc(Rc_Get_SBUS.PITCH,50)	;
		ocu.sbus_rc_main[3]=Rc_Get.YAW=my_deathzoom_rc(Rc_Get_SBUS.YAW,50)	;
		ocu.sbus_aux[0]=Rc_Get.AUX1=Rc_Get_SBUS.AUX1;
		ocu.sbus_aux[1]=Rc_Get.AUX2=Rc_Get_SBUS.AUX2;
		ocu.sbus_aux[2]=Rc_Get.AUX3=Rc_Get_SBUS.AUX3;
		ocu.sbus_aux[3]=Rc_Get.AUX4=Rc_Get_SBUS.AUX4;
		
		ocu.sbus_power_sw=(ocu.sbus_rc_main[0]<1100&&ocu.sbus_rc_main[1]>1800&&ocu.sbus_rc_main[3]<1100);
		
		ocu.sbus_mode=ocu.sbus_aux[0];
		ocu.sbus_height=LIMIT((ocu.sbus_aux[1]-1500)/500.,-1,1);
		ocu.sbus_mode_e=ocu.sbus_aux[2];
		
		if(vmc_all.gait_mode==STAND_RC){
		ocu.rc_spd_w[Xr]=0;
		ocu.rc_spd_w[Yr]=0;
		ocu.rate_yaw_w=LIMIT((Rc_Get_SBUS.YAW-1500)/500.,-1,1);
		ocu.rc_att_w[Xr]=LIMIT((Rc_Get.PITCH-1500)/500.,-1,1);
		ocu.rc_att_w[Yr]=-LIMIT((Rc_Get.ROLL-1500)/500.,-1,1);
		}
		else{
		ocu.rc_spd_w[Xr]=LIMIT((Rc_Get.THROTTLE-1500)/500.,-1,1);
		ocu.rc_spd_w[Yr]=LIMIT((Rc_Get.YAW-1500)/500.,-1,1);
		ocu.rate_yaw_w=-LIMIT((Rc_Get.ROLL-1500)/500.,-1,1);
		ocu.rc_att_w[Xr]=LIMIT((Rc_Get.PITCH-1500)/500.,-1,1);
		ocu.rc_att_w[Yr]=0;				
		}
	}
	 
	if(Rc_Get_SBUS.lose_cnt++>2/0.05)Rc_Get_SBUS.connect=0;
	if(o_cmd.lost_cnt++>125)o_cmd.connect=0;
	if(ocu.loss_cnt++>2/0.05)ocu.connect=ocu.mode=0;
	if(ocu_loss_cnt++>2/0.05)ocu_connect=0;
  if(spi_master_loss_pi++>2/0.05){spi_master_connect_pi=0;spi_master_loss_pi_all++;}
	if(palm_dj.loss_cnt++>2/0.05)palm_dj.connect=0;
 
	for(i=0;i<14;i++){
		leg_motor.connect_motor[i]=motor_chassis[i].param.connect;
		if(motor_chassis[i].param.loss_cnt++>0.5/0.05)
			motor_chassis[i].param.connect=0;
	}

	if(imuo.cnt_loss++>0.5/0.02)imuo.connect=0;

	static char beep_state=0;
	static float beep_timer=0;
	
	#if defined(EN_BEEP)
		if(robotwb.beep_state!=0){

		}
		else if(beep_state==0&&spi_master_connect_pi)
		{
				beep_timer=0;Reset_Beep_Task();
		}
		
		switch(beep_state)
		{
			case 0:
				Reset_Beep_Task();
				beep_timer+=0.05;
			if(beep_timer>0.1)
			{
				beep_timer=0;
				beep_state++;
			}
			break;
			case 1:
				if(robotwb.beep_state==BEEP_BLDC_ZERO_CAL){
					beep_timer=0;Reset_Beep_Task();
					beep_state=BEEP_BLDC_ZERO_CAL;
				}else if(robotwb.beep_state==BEEP_BLDC_ZERO_INIT){
					beep_timer=0;Reset_Beep_Task();
					beep_state=BEEP_BLDC_ZERO_INIT;
			  }else if(robotwb.beep_state==BEEP_BLDC_GAIT_SWITCH){
					beep_timer=0;Reset_Beep_Task();
					beep_state=BEEP_BLDC_GAIT_SWITCH;
				}else if(robotwb.beep_state==BEEP_BLDC_RESET_ERR){
					beep_timer=0;Reset_Beep_Task();
					beep_state=BEEP_BLDC_RESET_ERR;
				}
				else
					Play_Music_Task(BEEP_BLDC_STATE,0.05);
			break;
			case BEEP_BLDC_ZERO_CAL:
				if(Play_Music_Task(BEEP_BLDC_ZERO_CAL,0.05))
				{
					beep_timer=0;
					robotwb.beep_state=beep_state=0;
					Reset_Beep_Task();
				}
			break;
			case BEEP_BLDC_ZERO_INIT:
				if(Play_Music_Task(BEEP_BLDC_ZERO_INIT,0.05))
				{
					beep_timer=0;
					robotwb.beep_state=beep_state=0;
					Reset_Beep_Task();
				}
			break;
					case BEEP_BLDC_GAIT_SWITCH:
				if(Play_Music_Task(BEEP_BLDC_GAIT_SWITCH,0.05))
				{
					beep_timer=0;
					robotwb.beep_state=beep_state=0;
					Reset_Beep_Task();
				}
			break;				
					case BEEP_BLDC_RESET_ERR:
				if(Play_Music_Task(BEEP_BLDC_RESET_ERR,0.05))
				{
					beep_timer=0;
					robotwb.beep_state=beep_state=0;
					Reset_Beep_Task();
				}
			break;
			case BEEP_BLDC_SPI_CONNECT:
			if(Play_Music_Task(BEEP_BLDC_SPI_CONNECT,0.05))
			{
				beep_timer=0;
				robotwb.beep_state=beep_state=0;
				Reset_Beep_Task();
			}
			break;		
			case BEEP_BLDC_SPI_CONNECT_THREAD_UP:
			if(Play_Music_Task(BEEP_BLDC_SPI_CONNECT_THREAD_UP,0.05))
			{
				beep_timer=0;
				robotwb.beep_state=beep_state=0;
				Reset_Beep_Task();
			}
			break;					
		}
		#endif
}



int time_scale=1;
void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{
	int id=0;				
	static u8 mav_state;
	static u16 cnt[3];
	static int cnt_1ms=0;
	static int extfb_flag=0;
	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;

		if( loop.cnt_2ms >= 2*time_scale )//周期2ms的任务 500Hz
		{
			loop.cnt_2ms = 0;						
		}

		if(!spi_master_connect_pi)//掉线保护
		{
			for(id=0;id<14;id++){
				leg_motor.set_t[id]=0;
				leg_motor.q_set[id]=leg_motor.q_now[id];
				leg_motor.motor_en=0;
			}
		}
		
		Duty_Servo();							

		Duty_Link();		
			
		if( loop.cnt_5ms >= 5 )//周期5ms的任务 200Hz
		{
	
			loop.cnt_5ms = 0;		
		}
		
		if( loop.cnt_10ms >= 5 )//周期10ms的任务 100Hz 轮毂控制
		{
			loop.cnt_10ms = 0;
		}

		if( loop.cnt_20ms >= 25*time_scale &&1)//周期20ms的任务 50Hz 舵机控制
		{
			loop.cnt_20ms = 0;
			float dt_dj=Get_Cycle_T(14); 	
			serial_servo(dt_dj);
			
			if(extfb_flag==0){
				extfb_flag=1;
				send_fb1_extcan();//  fb
			}else if(extfb_flag==1){
				extfb_flag=2;
				send_fb2_extcan();//  fb			
			}else if(extfb_flag==2){
				extfb_flag=0;
				send_fb3_extcan();//  fb			
			}
		}

		if( loop.cnt_50ms >= 50*time_scale )
		{

			loop.cnt_50ms = 0;
			Duty_System();
				
		}
		//LED控制
		if( loop.cnt_1s >= 500*time_scale )
		{
			loop.cnt_1s = 0;
			can_rx_over[4]=0;		
		}
		
		loop.check_flag = 0;		//循环运行完毕标志
	}
}
