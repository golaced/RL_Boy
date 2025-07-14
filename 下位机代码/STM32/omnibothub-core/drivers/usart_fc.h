#ifndef _USART_H
#define _USART_H

#include "stm32f4xx.h"
typedef struct 
{
	float x;
	float y;
	float z;
	float zz;
}END_POSS;


extern u8 KEY[8];

typedef struct 
{
	float ip1;
	float ip2;
	float extcan_link;
	float spi_link;
	float can1_link;
	float can2_link;
}_Robot;

extern _Robot robot;

void Usart1_Init(u32 br_num);
void Usart2_Init(u32 br_num);
void Usart4_Init(u32 br_num);
void Usart3_Init(u32 br_num);
void Uart5_Init(u32 br_num);
void Uart6_Init(u32 br_num);
typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t MOV1,MOV2;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
				int16_t AUX6;
				float SBUS_CH[6];
				int16_t AUX_SEL1,AUX_SEL2,AUX_SEL3,AUX_SEL4,AUX_SEL5,AUX_SEL6;
	      int16_t HEIGHT_MODE;
	      int16_t POS_MODE;
	      u8 update,Heart,Heart_rx,Heart_error;
	      int  lose_cnt,lose_cnt_rx;
	      u8 connect;
				float signal_rate;
				int16_t RST;}RC_GETDATA;

extern RC_GETDATA Rc_Get,Rc_Get_PWM,Rc_Get_SBUS,Rc_Wifi;//接收到的RC数据,1000~2000
						

typedef struct
{
  int connect;
  char power;
  int cap;
  int mode;
  int loss_connect;
  float att_set[3];
  END_POSS pos_set;
  float att_now[3];
  END_POSS pos_now;
	float dis;
  float move_spd;
  float torque;
} _ARMSS;

extern _ARMSS arm_cmd_s;
void Send_To_ARM(void);

typedef struct
{
  int connect;
  int cnt_loss;
	float dt;
	float att[3];
	float rate[3];
	float acc_b[3];
	float acc_n[3];
} _IMUO;

extern _IMUO imuo;

typedef struct{
	char check;
	 int x;//目标的x坐标
	 int y;//目标的y坐标
	 int w;//目标的宽度
	 int h;//目标的高度
	 int s;
	float angle;
	float pos[3],att[3];
}_PIX_TAR;

typedef struct{
	_PIX_TAR cube,color,face,line;
	float target_pos[3];
	float target_att[3];
	
	float cmd_spd[3];
	float cmd_pos[3];
	float cmd_att[4];
	u8 power;
	u8 cmd_mode;
	u8 visual_mode;
	u8 connect;
	u16 lost_cnt;
}_PI;//识别结果

extern _PI pi;

typedef struct{
	int origin[2];
	float spd_o[2];
	float spd_flt[2];
	float dis;
	u8 mode;
	u8 connect;
	u16 lost_cnt;
}_ODOMETER;//识别结果

extern _ODOMETER flow;

typedef struct{
	u8 mode;
	u8 connect;
	u16 lost_cnt;
	float pos[3],spd[3],att[3];
}_LINK_CMD;

extern _LINK_CMD o_cmd;

typedef struct{
	u8 mode;
	u8 connect;
	u16 loss_cnt;
	u8 type;
	int pix[3],spd_o[3];
	float ang[3],out[3];
	float spd[3],spd_compass[3],height;
	float spd_flt[3],height_flt;
	float spdo[3],out_reg[3],spdo_flt[3];
	float k_all,cut_rate;
	float k_att_compass[3];
}_FLOW;

extern _FLOW optical_flow;
extern _LINK_CMD o_cmd;
#define SEND_BUF_SIZE1 64+32	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.250	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff1[SEND_BUF_SIZE1];	//发送数据缓冲区
extern u8 SendBuff1_cnt;
extern u8 SendBuff3_cnt;
extern u8 SendBuff6_cnt;

void UsartSend1(uint8_t ch);
void UsartSend4(uint8_t ch);
void data_per_uart1(void);
void data_per_uart1_bldc(void);
void data_per_uart1_bldc_param(void);
void clear_leg_uart(void);
void clear_pi_uart(void);
extern u16 leg_uart_cnt,leg_pi_cnt;
char isRxCompleted(void);
void radio_control(char sel,float dt);
void radio_off(void);
#define SEND_BUF_SIZE2 40	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff2[SEND_BUF_SIZE2];	//发送数据缓冲区
void data_per_uart2(void);

#define SEND_BUF_SIZE3 100	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff3[SEND_BUF_SIZE3];	//发送数据缓冲区
void data_per_uart3(u8 sel);

#define SEND_BUF_SIZE4 64	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff4[SEND_BUF_SIZE4];	//发送数据缓冲区
void data_per_uart4(u8 sel);

#define SEND_BUF_SIZE6 100	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
extern u8 SendBuff6[SEND_BUF_SIZE6];	//发送数据缓冲区

extern int16_t BLE_DEBUG[16];
void data_per_uart_rc(int16_t ax,int16_t ay, int16_t az, int16_t gx,int16_t  gy, int16_t gz,int16_t hx, int16_t hy, int16_t hz,
	int16_t yaw,int16_t pitch,int16_t roll,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);				

void Send_buf1(u8 *dataToSend , char length);
#define MODE_CUBE 1
#define MODE_FACE 2
#define MODE_COLOR 3
#define MODE_LINE 4
#define MODE_CMD  11
#include "nlink_linktrack_aoa_nodeframe0.h"
#include "nlink_utils.h"
uint8_t Aoa_UnpackData(const uint8_t *data, int data_length);

//mavlink

void clear_nrf_uart(void);
typedef struct
{
 float Pit,Rol,Yaw;
 double Lat,Lon;
 float H,H_Spd,H_G;	
 float q[4];
 u8 GPS_STATUS;	//>3  5->Best
 float X_Spd_b,Y_Spd_b,X_Spd_n,Y_Spd_n;
 int Dop[6];
/*
Flight status val	status name
1	standby
2	take_off
3	in_air
4	landing
5	finish_landing
*/
 u8 STATUS;
 float Bat;	
 int Rc_pit,Rc_rol,Rc_yaw,Rc_thr,Rc_mode,Rc_gear;
 u8 m100_connect,m100_data_refresh;
 double Init_Lat,Init_Lon;
 float Y_Pos_local,X_Pos_local,spd[3];
 long r1,r2;
 float rx_dt;
 u8 control_connect,connect;
 u16 control_loss,cnt_m100_data_refresh,loss_cnt;
 u8 px4_tar_mode;
 float control_spd[3];
 float control_yaw;
 float uwb_o[3];
 u8 save_data,save_video;
 u8 navigation_mode;
 float uwb_f[4];//x y z yaw
}M100;
extern M100 m100,px4;

//--------------
#define WHEEL_ID_OFF 10
void wheel_motor_loop(char id,float dt);
char PY_CRC_8_T(char *di, uint32_t len);
typedef struct
{
 u8 connect;
 u8 mode;
 int cmd_flag;
 float rad_set;	
	
 float current;
 float tau_now;
 float rad_now;
 float q_now;
 float v_now;
 char err;
 float r;
 float dt_rx;
 float i_2_tau;
 int rx_cnt;
 int loss_cnt;
}_WHEEL_WX;

typedef struct
{
  float w_set;
	float v_set;
	float v_set_flt;
	float w_now,w_imu;
	float v_now;
	float r;
	float x_n,yn;
	float vx_n,vy_n;
	float h;
	float ws;
	float w_set_flt,w;
}_WHEEL_2Dof;

extern _WHEEL_2Dof _wheel_2d;
extern _WHEEL_WX _wheel_wx[4];
extern float ws_set_flt;
void wheel_2d_loop(float dt);

extern u8 RxState1;
//-----------------DJ
extern int pwm_dj[5];
extern int time_dj[5];
void set_dj_pwm(int id,int pwm,int time);
void set_dj_pwm_all(int pwm[5],int time[5]);
void dj_control_loop(float dt);

typedef struct
{
	char connect_link;
	int loss_cnt;
	char connect[14];
	char power,cal_div[14];
  float head_att[3];
	float hand_att[3];
	float hand_att_test[3];	
	float cap_rate;
	float cap_touch;
	float stiff[14];
	float q_now[14],dq_now[14],t_now[14];
	float q_exp[14],t_exp[14],q_reset[14];
	float kp[14],kd[14];
	int inv_flag[14];
	int off_set_pwm[14];
	float degree_to_pwm[14];
	
}_Palm_DJ;
extern _Palm_DJ palm_dj;
	

//-------------------飞特舵机

enum SCS_type
{
  CUSTOM_SCS=0, SCS0009, SCS00091, SCS00092, STS3215, STS3046
};


typedef struct 
{
	char bus_id;
	enum SCS_type type;
	char en,connect;
	int POS,POS_OFF;
	int POS_OFF_SET;
	uint16_t POS_SET;
	int LOAD;
	float exp_q,q;
	float exp_dq,dq;
	float exp_t,t;
	float reset_angle;
	int t_inv_cmd;
	int t_measure_cmd;
	int q_inv;
	
	float imp_kp;
	float imp_kd;
	float imp_stiff;
	
	float MAX_AD_Q;
	float AD_Q;

	char  END_S;
	char sms_type;
}_UART_SERVO;
extern _UART_SERVO servo_s[14],servo_ss[14];
extern char END_SCS_ALL;
void serial_init(void);
void serial_power_off_all(void);
void serial_power_off_joint(char id);
void serial_set_q(char id,float q_set);
void serial_servo_feed_back(char id);
void serial_set_q_smooth(char id,float q_set,float gain_spd,float dt);
void readOnlyss(void);
void sendOnlyss(void);
void serial_servo(float dt);

void Uart_Flush(void);
int16_t Uart_Read(void);
void Uart_Send(uint8_t *buf , uint8_t len);

void send_cmd1_extcan(void);
void send_cmd2_extcan(void);
void send_cmd3_extcan(void);

//Audio
void Write_Audio_Data(uint8_t dat);
void Audio_system();
void Audio_bldc();
#endif
