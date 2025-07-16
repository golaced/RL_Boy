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
#include "test.h"
#include "mavl.h"
#include "spi.h"
#include "gps.h"
#include "usbd_cdc_vcp.h" 
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
	loop.cnt_2s++;
	if(loop.time<0)loop.time=0;
	if(loop.cnt_2ms<0)loop.cnt_2ms=0;
	if(loop.cnt_5ms<0)loop.cnt_5ms=0;
	if(loop.cnt_10ms<0)loop.cnt_10ms=0;
	if(loop.cnt_20ms<0)loop.cnt_20ms=0;
	if(loop.cnt_50ms<0)loop.cnt_50ms=0;
	if(loop.cnt_1s<0)loop.cnt_1s=0;
	if(loop.cnt_2ms<0)loop.cnt_2ms=0;
	if(loop.cnt_5ms<0)loop.cnt_5ms=0;
	if(loop.cnt_10ms<0)loop.cnt_10ms=0;
	if(loop.cnt_20ms<0)loop.cnt_20ms=0;
	if(loop.cnt_50ms<0)loop.cnt_50ms=0;
	if(loop.cnt_2s<0)loop.cnt_2s=0;
	
	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
	if(loop.err_flag>9999)loop.err_flag=0;
	
}

void Duty_Servo()//1ms 伺服驱动
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

void Duty_Att_Fushion()//姿态解算 100Hz
{  
	u8 i;
	static u8 init;	
	static u16 cnt_init;
	static float timer_baro;
	static float att_rt_use[3];
	float T;
	if(!init){
		init=1;
	}
	system_dt.ahrs_task=T=leg_dt[3] = Get_Cycle_T(3); 	

	if(!module.flash_lock&&!imuo.connect){
	IMU_Read(); 														
	IMU_Data_Prepare( T );		
	}
		
	if(cnt_init++>1/T&&!imuo.connect){cnt_init=65530;
		#if defined(ATT_MAD)//&&!defined(BOARD_FOR_CAN)
			madgwick_update_new(T,
			mems.Gyro_deg_rt.x/57.3, mems.Gyro_deg_rt.y/57.3,mems.Gyro_deg_rt.z/57.3, 
			mems.Acc_rt.x, mems.Acc_rt.y, mems.Acc_rt.z,
			mems.Mag_rt.x*module.hml_imu*mems.Mag_Have_Param*0,
			mems.Mag_rt.y*module.hml_imu*mems.Mag_Have_Param*0,
			mems.Mag_rt.z*module.hml_imu*mems.Mag_Have_Param*0,
			&Pitch,&Roll,&Yaw);
		#endif
		#if defined(ATT_COM)//&&!defined(BOARD_FOR_CAN)
			IMUupdate(T/2,
			mems.Gyro_deg_rt.x, mems.Gyro_deg_rt.y,mems.Gyro_deg_rt.z, 
			mems.Acc_rt.x, mems.Acc_rt.y, mems.Acc_rt.z,
			&Pitch,&Roll,&Yaw);
		#endif
		#if defined(ATT_EKF)//||defined(BOARD_FOR_CAN)
			ahrs_ekf.acc_m[0]=(float) mems.Acc_rt.x/4096.*9.8;
			ahrs_ekf.acc_m[1]=(float) mems.Acc_rt.y/4096.*9.8;
			ahrs_ekf.acc_m[2]=(float) mems.Acc_rt.z/4096.*9.8;
			ahrs_ekf.gyro_m[0]=(float) mems.Gyro_deg_rt.x/57.3;
			ahrs_ekf.gyro_m[1]=(float) mems.Gyro_deg_rt.y/57.3;
			ahrs_ekf.gyro_m[2]=(float) mems.Gyro_deg_rt.z/57.3;	
			#if EN_GYRO_Z_F_ODOM
				float err_z=fabs(my_deathzoom(mems.Gyro_deg_rt.z-vmc_all.att_rate_vm[YAWr],5));
				float yaw_f_weight=LIMIT(err_z,0,15)/15;
				yaw_f_weight=LIMIT(yaw_f_weight+0.0,0,1);
				ahrs_ekf.gyro_m[2]=(float)(mems.Gyro_deg_rt.z*yaw_f_weight+(1-yaw_f_weight)*vmc_all.att_rate_vm[YAWr])/57.3;	
			#endif
			EKF_AHRS_UPDATE(T);
			Pitch=ahrs_ekf.att_f_use[0];
			Roll=ahrs_ekf.att_f_use[1];
			Yaw=ahrs_ekf.att_f_use[2]-mems.imu_att.z;
		#endif
  float a_br[3],acc_temp[3];
	static float acc_flt[3];
	a_br[0] =(float) mems.Acc_rt.x/4096.;
	a_br[1] =(float) mems.Acc_rt.y/4096.;
	a_br[2] =(float) mems.Acc_rt.z/4096.;
	acc_temp[0] = a_br[1]*reference_vr[2]  - a_br[2]*reference_vr[1] ;
	acc_temp[1] = a_br[2]*reference_vr[0]  - a_br[0]*reference_vr[2] ;
	acc_temp[2] = reference_vr[2] *a_br[2] + reference_vr[0] *a_br[0]+ reference_vr[1] *a_br[1] - 1 ;
	
	vmc_all.acc_nn.x=acc_temp[0]*9.81;
	vmc_all.acc_nn.y=acc_temp[1]*9.81;
	vmc_all.acc_nn.z=acc_temp[2]*9.81;
	
	#if defined(CODE_VER2)
	vmc_all.att[PITr]= Pitch-vmc_all.tar_att_bias[PITr]*0;
	vmc_all.att[ROLr]=-Roll -vmc_all.tar_att_bias[ROLr]*0;
	vmc_all.att[YAWr]= Yaw;		
	#else
	vmc_all.att[PITr]=Pitch;
	vmc_all.att[ROLr]=-Roll;
	vmc_all.att[YAWr]=Yaw;		
	#endif
	vmc_all.acc_b.x=a_br[0];
	vmc_all.acc_b.y=a_br[1];
	vmc_all.acc_b.z=a_br[2];
	
	vmc_all.acc_n.x=-vmc_all.acc_b.x*sind(vmc_all.att[YAWr])+vmc_all.acc_b.y*cosd(vmc_all.att[YAWr]);
	vmc_all.acc_n.y= vmc_all.acc_b.x*cosd(vmc_all.att[YAWr])+vmc_all.acc_b.y*sind(vmc_all.att[YAWr]);
		
	DigitalLPF(vmc_all.att[PITr],&att_rt_use[PITr],FLT_ATT_RT,T);
	DigitalLPF(vmc_all.att[ROLr],&att_rt_use[ROLr],FLT_ATT_RT,T);
	DigitalLPF(vmc_all.att[YAWr],&att_rt_use[YAWr],FLT_ATT_RT,T);
//	if(fabs(vmc_all.att[ROLr])>15)
//		att_rt_use[ROLr]=0;
	att_rt_use[YAWr]=0;
	vmc_all.Rn_b[0][0] =  cosd(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]); 
	//cy cz
	vmc_all.Rn_b[1][0] = -cosd(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr])+sind(-att_rt_use[PITr])*sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr])  ;
	//-cx sz + sy sx cz
	vmc_all.Rn_b[2][0] =  sind(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr])+cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr])  ;
	//sx sz + cx sy cz
	
	vmc_all.Rn_b[0][1] =  cosd(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);			
//cy sz	
	vmc_all.Rn_b[1][1] =  cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr])+sind(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr])  ;	
//cx cz + sx sy sz	
	vmc_all.Rn_b[2][1] = -sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr])+cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr])  ;			
	//-sx cz + cx sy sz
	
	vmc_all.Rn_b[0][2] = -sind(-att_rt_use[PITr]);
	//-sy
	vmc_all.Rn_b[1][2] =  sind(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]); 
	//sx cy
	vmc_all.Rn_b[2][2] =  cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
	//cx cy

	mat_trans(vmc_all.Rn_b,vmc_all.Rb_n);
	att_rt_use[ROLr]=0;
	att_rt_use[YAWr]=0;
	vmc_all.Rn_b_noroll[0][0] =  cosd(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]); 
	vmc_all.Rn_b_noroll[1][0] = -cosd(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr])+sind(-att_rt_use[PITr])*sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr])  ;
	vmc_all.Rn_b_noroll[2][0] =  sind(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr])+cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr])  ;
	
	vmc_all.Rn_b_noroll[0][1] =  cosd(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);										 
	vmc_all.Rn_b_noroll[1][1] =  cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr])+sind(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr])  ;																
	vmc_all.Rn_b_noroll[2][1] = -sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr])+cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr])  ;			
	
	vmc_all.Rn_b_noroll[0][2] = -sind(-att_rt_use[PITr]);
	vmc_all.Rn_b_noroll[1][2] =  sind(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]); 
	vmc_all.Rn_b_noroll[2][2] =  cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
	
	mat_trans(vmc_all.Rn_b_noroll,vmc_all.Rb_n_noroll);
	
	#if VIR_MODEL
	  vmc_all.att[YAWr]=nav.fake_yaw;
	#endif
	#if defined(ATT_EKF)
		vmc_all.att_rate[PITr]=ahrs_ekf.gyro_f_use[0]*57.3;
		vmc_all.att_rate[ROLr]=ahrs_ekf.gyro_f_use[1]*57.3;
		vmc_all.att_rate[YAWr]=ahrs_ekf.gyro_f_use[2]*57.3;
	#elif defined(ATT_EKF_Q)
		vmc_all.att_rate[PITr]=INS.Gyro_fix[0]*57.3;
		vmc_all.att_rate[ROLr]=INS.Gyro_fix[1]*57.3;
		vmc_all.att_rate[YAWr]=INS.Gyro_fix[2]*57.3;
	#else
		vmc_all.att_rate[PITr]=mems.Gyro_deg_rt.x;
		vmc_all.att_rate[ROLr]=mems.Gyro_deg_rt.y;
		vmc_all.att_rate[YAWr]=mems.Gyro_deg_rt.z;
	#endif
	DigitalLPF( acc_temp[0]*9.8, &vmc_all.acc[Xr], FLT_ACC, T);
	DigitalLPF(-acc_temp[1]*9.8, &vmc_all.acc[Yr], FLT_ACC, T);
	DigitalLPF( acc_temp[2]*9.8, &vmc_all.acc[Zr], FLT_ACC, T);

	subscribe_imu_to_webot(&robotwb, T);//赋值给robot结构体
  }				
}

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
	
	//板载LED
	LEDRGB_STATE(0.05);
	//扩展LED显示机器人状态
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
	if(Rc_Get_SBUS.update)//
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
		
		ocu.rc_spd_w[Xr]=0;
		ocu.rc_spd_w[Yr]=0;
		ocu.rate_yaw_w=LIMIT((Rc_Get_SBUS.YAW-1500)/500.,-1,1);
		ocu.rc_att_w[Xr]=LIMIT((Rc_Get.PITCH-1500)/500.,-1,1);
		ocu.rc_att_w[Yr]=-LIMIT((Rc_Get.ROLL-1500)/500.,-1,1);
	}
	 
	if(Rc_Get_SBUS.lose_cnt++>2/0.05)Rc_Get_SBUS.connect=0;
	if(o_cmd.lost_cnt++>125)o_cmd.connect=0;
	if(ocu.loss_cnt++>2/0.05)ocu.connect=ocu.mode=0;
	if(ocu_loss_cnt++>2/0.05)ocu_connect=0;
  if(spi_master_loss_pi++>2/0.05){spi_master_connect_pi=0;spi_master_loss_pi_all++;}
 
	for(i=0;i<14;i++){
		leg_motor.connect_motor[i]=motor_chassis[i].param.connect;
		if(motor_chassis[i].param.loss_cnt++>0.5/0.05)
			motor_chassis[i].param.connect=0;
	}
	robot.spi_link=spi_master_connect_pi;
	//robot.extcan_link=palm_dj.connect_link;
	robot.extcan_link=0;
	if(palm_dj.connect_link)
		for(i=0;i<14;i++)
			robot.extcan_link+=palm_dj.connect[i];
	else
		robot.extcan_link=0;
	robot.can1_link=0;robot.can2_link=0;
	for(int i=0;i<7;i++){
		if(leg_motor.connect_motor[i]==1)
			robot.can1_link++;
	}
	for(int i=7;i<14;i++){
		if(leg_motor.connect_motor[i]==1)
			robot.can2_link++;
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
	static int ext_send_flag=0;
	static int cnt_1ms=0;
	static int ip_get=0;
	static float timer_ip=0;
	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		#if SPI_PI_IS_MASTER
			master_send();
		#endif
		
		if( loop.cnt_2ms >= 2*time_scale )//周期2ms的任务 500Hz
		{
			Duty_Att_Fushion();//状态估计 + VMC力控		
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
			#if MCU_TINYPALE||MCU_TINYPALE_S1
			wheel_2d_loop(0.01);
			#endif
			#if MCU_TINYPALE_S1//控制轮子
			static int flag_motor=0;
		 	if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA2_Steam7传输完成
			{ 
				DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//清除DMA2_Steam7传输完成标志
				for	(SendBuff1_cnt=0;SendBuff1_cnt<SEND_BUF_SIZE1;SendBuff1_cnt++)
					SendBuff1[SendBuff1_cnt]=0;
				SendBuff1_cnt=0;
				float dt_wheel=Get_Cycle_T(14); 	
				if(flag_motor)
				wheel_motor_loop(1,dt_wheel);
				else
				wheel_motor_loop(2,dt_wheel);
				flag_motor=!flag_motor;
				USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
				MYDMA_Enable(DMA1_Stream6,SendBuff1_cnt+2);     //开始一次DMA传输！	
			}	
			#endif
			#if MCU_TINYPALE
			static int flag_motor=0;
		  if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
			{ 	
				DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
				for	(SendBuff1_cnt=0;SendBuff1_cnt<SEND_BUF_SIZE1;SendBuff1_cnt++)
					SendBuff1[SendBuff1_cnt]=0;
				SendBuff1_cnt=0;
				float dt_wheel=Get_Cycle_T(14); 	
				wheel_motor_loop(flag_motor++,dt_wheel);
				if(flag_motor>4)
					flag_motor=0;
				USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
				MYDMA_Enable(DMA2_Stream7,SendBuff1_cnt);     //开始一次DMA传输
				RxState1 = 0;
			}	
			#endif
		}
		
		#if 0//舵机机械臂控制 钟灵舵机
	  if( loop.cnt_20ms >= 20*time_scale &&1)//周期20ms的任务 50Hz
		{
			loop.cnt_20ms = 0;	
			if(DMA_GetFlagStatus(DMA2_Stream6,DMA_FLAG_TCIF6)!=RESET)//等待DMA2_Steam7传输完成
			{ 	
				DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);//清除DMA2_Steam7传输完成标志
				for	(SendBuff6_cnt=0;SendBuff6_cnt<SEND_BUF_SIZE6;SendBuff6_cnt++)
					SendBuff6[SendBuff6_cnt]=0;
				SendBuff6_cnt=0;
				float dt_dj=Get_Cycle_T(14); 	
				dj_control_loop(dt_dj);
				USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
				MYDMA_Enable(DMA2_Stream6,SendBuff6_cnt+2);     //开始一次DMA传输！	  
			}	
		}
		#else
		#if USE_VR
			if( loop.cnt_20ms >= 10*time_scale &&1)//周期20ms的任务 100Hz 飞特舵机控制
			{
		#else
			if( loop.cnt_20ms >= 20*time_scale &&1)//周期20ms的任务 50Hz 飞特舵机控制
			{
		#endif
			loop.cnt_20ms = 0;
			float dt_dj=Get_Cycle_T(14); 	
			#if !USE_OLED||USE_SERVO1||USE_VR
				serial_servo(dt_dj);
			#endif 
			#if EN_DMA_UART3//omnihub-extcan unuse now 目前DMA被SPI占用
				if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)
				{ 	
					DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);
					for	(SendBuff3_cnt=0;SendBuff3_cnt<SEND_BUF_SIZE3;SendBuff3_cnt++)
						SendBuff3[SendBuff3_cnt]=0;
					SendBuff3_cnt=0;
					//your code here
					
					
					USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  
					MYDMA_Enable(DMA1_Stream3,SendBuff3_cnt+2);
				}		
			#else
				send_cmd1_extcan();
			#endif
		}
		#endif	
		
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
			if(ext_send_flag==0){
				ext_send_flag=1;
				send_cmd2_extcan();	
			}else if(ext_send_flag==1){
				ext_send_flag=0;
				send_cmd3_extcan();	
			}
		}
		timer_ip+=Get_Cycle_T(25); 	
		if( timer_ip>2 )
		{
			timer_ip = 0;		
			if(spi_master_connect_pi==1&&robot.ip1!=0
				&&!ip_get
			){
				ip_get=1;
				
				IWDG_Init(4,25000);//100ms
				
				OLED_Show();
				#if USE_AUDIO
					Audio_system();
				#endif
				IWDG_Init(4,250);//100ms
		  }
			//wslled_loop(0.5);//PWM LED
		}
		
		loop.check_flag = 0;		//循环运行完毕标志
	}
}
