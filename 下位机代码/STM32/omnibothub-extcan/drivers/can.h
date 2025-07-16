#ifndef __CAN1_H
#define __CAN1_H	 
#include "sys.h"	 
#include "stm32f4xx_can.h" 
#define CAN_ANL_MIT_MODE 
//主控
#define CAN_FB_SYNC 1//同步采用
#define CAN_NART_SEL DISABLE//ENABLE//DISABLE//ENABLE
//DISABLE  底层节点没法收到  但是能保证不加延时
#define BAUD_2M 1


#define CAN_T_DIV 500.0
#define CAN_I_DIV 100.0
#define CAN_F_DIV 100.0
#define CAN_POS_DIV 50.0
#define CAN_DPOS_DIV 20.0
#define CAN_GAIN_DIV_P_M 10.0
#define CAN_GAIN_DIV_I_M 1000.0
#define CAN_GAIN_DIV_D_M 100.0
#define CAN_GAIN_DIV_P 500.0
#define CAN_GAIN_DIV_I 10000.0
#define CAN_GAIN_DIV_D 1000.0

#define CAN_SD_PARAM_SYS_HEAD  0
#define CAN_SD_PARAM_SYS_HEAD_DIV  10
#define CAN_SD_PARAM_T_HEAD  20
#define CAN_SD_PARAM_T_HEAD_GAIN  40
#define CAN_SD_PARAM_T_HEAD_SW  50
#define CAN_SD_PARAM_T_HEAD_ST  60

#define CAN_FB_STATE_HEAD  70
#define CAN_FB_POS_HEAD  80
#define CAN_FB_SPD_HEAD  90
#define CAN_FB_MIT_HEAD  100

#define CAN_SD_T_HEAD  120
#define CAN_SD_I_HEAD  140
#define CAN_SD_POS_HEAD  160
#define CAN_SD_SPD_HEAD  180
#define CAN_SD_MIT_HEAD  200

#define CAN_SD_MIT_HEAD_Q  210
#define CAN_SD_MIT_HEAD_T  220

#define CAN_SD_PARAM_T_HEAD_SW_D1  300
#define CAN_SD_PARAM_T_HEAD_SW_D2  310
#define CAN_SD_PARAM_T_HEAD_SW_D3  320
#define CAN_SD_PARAM_T_HEAD_ST_D1  330
#define CAN_SD_PARAM_T_HEAD_ST_D2  340
#define CAN_SD_PARAM_T_HEAD_ST_D3  350

#define CAN_FB_REMOTE_FB1      400
#define CAN_FB_REMOTE_FB2      401
#define CAN_FB_REMOTE_FB1_Q3   402
#define CAN_FB_REMOTE_FB1_I3   403
#define CAN_FB_REMOTE_FB1_DQ3   404
#define CAN_FB_REMOTE_RST_Q    412
#define CAN_FB_REMOTE_RST_ERR  413

#define CAN_FB_POS3_HEAD  430
#define CAN_FB_I3_HEAD  440
#define CAN_FB_STATE_HEAD_Q3  450
#define CAN_FB_DPOS3_HEAD  460
#define CAN_FB_DPOSW_HEAD  500

#define CAN_SD_PARAM_T_HEAD_WHEEL  510
#define CAN_SD_MIT_HEAD_WHEEL  520
#define CAN_FB_REMOTE_FB_WHEEL 530
#define CAN_FB_STATE_HEAD_WHEEL  540

#define P_MIN_MIT -180 //Degree
#define P_MAX_MIT  180
#define V_MIN_MIT -2000
#define V_MAX_MIT  2000
#define T_MIN_MIT -10.0f
#define T_MAX_MIT  10.0f
#define KP_MIN_MIT 0.0f
#define KP_MAX_MIT 10.0f
#define KD_MIN_MIT 0.0f
#define KD_MAX_MIT 10.0f

#define MIT_R_T_D 9.549
#define MIT_D_T_R 0.1047

//CAN1接收RX0中断使能
#define CAN_ABOM_E 1

#define CAN1_RX0_INT_ENABLE	1	//0,不使能;1,使能.								    
extern int can_rx_over[5];
extern int can_rx_cnt[5];							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,float brp,u8 mode);//CAN初始化
 
u8 CAN1_Send_Msg(u8* msg,u8 len,uint32_t id);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据
extern u8 canbuft1[8];
extern u8 canbufr1[8];
extern uint32_t can1_rx_id;
extern float cnt_rst1;
extern int can1_rx_cnt;
void data_can_anal_master(u8 rx_data[8]);

//CAN1接收RX0中断使能
#define CAN2_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
								 							 				    
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,float brp,u8 mode);//CAN初始化
 
u8 CAN2_Send_Msg(u8* msg,u8 len,uint32_t id);						//发送数据

u8 CAN2_Receive_Msg(u8 *buf);							//接收数据
extern u8 canbuft2[8];
extern u8 canbufr2[8];
extern uint32_t can2_rx_id;
extern float cnt_rst2;
extern int can2_rx_cnt;

#define MOTOR_MODE_POS 0
#define MOTOR_MODE_T	 1
#define MOTOR_MODE_CURRENT	2
typedef struct 
{
	char connect;
	char connect_motor[14];
	char ready[14];
  char connect_motor_w[4];
	char ready_w[4];
	int loss_cnt;
	char cal_div[14];
	char reset_q,reset_err;
	char motor_en,motor_en_reg;
	char motor_mode,motor_mode_reg;
	char err_flag[14];
	float q_now[14],qd_now[14],qdd_now[14];
	float q_set[14],qd_set[14],qdd_set[14];
	float t_now[14];
	float kp[14];
	float kd[14];
	float q_reset[14];
	float stiff[14];
	float set_t[14],set_t_flt[14];
	float set_i[14],set_i_flt[14];
	float set_t_w[4];
	float set_i_w[4];
	float dq_now_w[4];
	float t_now_w[4];
	float max_t[14];
	float max_t_w[4];
}_LEG_MOTOR;
extern _LEG_MOTOR leg_motor;

void CAN_set_torque(char id);
void CAN_reset_q(char id);
void CAN_motor_sm(float dt);
void CAN_motor_init(void);
void CAN_set_zero_off(char id);
extern char reset_err_flag;
void reset_current_cmd(char id);
void CAN_get_fb(char can_sel);//遥控反馈获取
extern  float can_dt[4];

enum Motor_type
{
  M_CUSTOM=0, M_DOGGO, M_2006, M_3508, M_MIT, TAIHU_1, GDM_FOC, GIM_4305,AK80_9 ,
	AK80_8, M_DIR, M_GO1, DM_J4310, M_RV, DIR_M15,AK10_9,DM_6006,DM_8006,EC_1,EC_2,EC_3
};

#define MODE_CURRENT 0
#define MODE_VEL 1
#define MODE_POS 2 
#define MODE_MIT 3
#define MODE_MIT_CURRENT 4

#define EN_MIT_PID_INNER 1 //内部PD模式
typedef struct{
	char id,init,connect;
	int loss_cnt;
	int usb_cmd_mode;
	int control_mode;
	float rx_dt;
	float q_reset_angle;
	int16_t	 	speed_rpm;
	int over_t_flag;
	float over_t_reg;
  float  	real_current;
  float  	given_current;
	float   given_torque;
	int32_t		round_cnt;
	int cnt_rotate;
	float set_q;
	float total_angle_out,total_angle_out_single,total_angle_out_reg;
	float total_angle_out_off;
	float t_scale;
	
	float q_now_reg;
	float qd_now_reg;
	float t_now_reg;
	
	float flt_q;
	float flt_qd;
	float flt_t;
	
	float bat_v;
	int t_inv_flag_measure;
	int t_inv_flag_cmd;
	int q_flag;
	int err_flag;
}motor_param_t;

typedef struct{
char ready;
char cal_zero_state;
float cal_zero_spd;
float cal_zero_thr;
char type;
char anal_type;
}motor_param_m;

typedef struct{
	char en_cmd,en_cmd_ocu;
	char reset_q,reset_q_rx,reset_q_en;
	char reset_q_lock;
	char reset_q_cnt;
	char cmd_mode,cal_div;
	float reset_q_delay_timer;
	char clear_err;
	char en_pos_trig;
	float set_q,set_q_test,set_q_test_bias;
	float set_q_mit_off;
	float set_qd;
	float set_t;
	float q_now,q_now_flt;
	float qd_now,qd_now_flt;
	float t_now,t_now_flt;
	float stiff;
	float kp;
	float ki;
	float kd;

	float max_t;
	float cmd_t,given_current_cmd,current_cmd_tx;
	
	motor_param_t param;
	motor_param_m motor;
}motor_measure_t;

extern motor_measure_t  motor_chassis[14];

void mit_bldc_thread(char en_all,float dt);
void data_can_mit_anal(motor_measure_t *ptr,uint8_t buf_rx[8]);
char data_can_mit_send(motor_measure_t *ptr);
char data_can_sample_only(motor_measure_t *ptr);
u8 mit_set_pos_zero( char id);
extern int can_write_flash,can_cmd_usb_disable;
#endif

















