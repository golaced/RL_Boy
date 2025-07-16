#ifndef _RC_MINE_H_
#define _RC_MINE_H_
#include "include.h"
#define RX_DR			6	
#define TX_DS			5
#define MAX_RT		4
#define MID_RC_KEY 15
#define MID_RC_GET 4
void Nrf_Check_Event(float dt);
void NRF_Send_AF(void);
void NRF_Send_AE(void);
void NRF_Send_OFFSET(void);
void NRF_Send_PID(void);
void NRF_Send_ARMED(void);
void NRF_SEND_test(void);
extern unsigned int cnt_timer2;
extern void RC_Send_Task(void);
extern u8 key_rc[6];
extern u16 data_rate;
extern void CAL_CHECK(void);


typedef struct 
{ 
  float spd[3];
	float pos[3];
	float acc_nez[3];
	float acc_body[3];
	float bat;
	u8 fly_mode;
	u8 acc_3d_step;
	u8 fall,knock;
}_DRONE;

extern _DRONE drone;
void param_copy_pid(void);
void pid_copy_param(void);

typedef struct 
{
 u8 gps,flow,sonar,bmp,flash,laser,pi,pi_flow,acc_imu,gyro_imu,hml_imu,hml_imu_o,nrf;
 u16 nrf_loss_cnt;
 u8 flash_lock;
}MOUDLE;
extern MOUDLE module;
#endif
