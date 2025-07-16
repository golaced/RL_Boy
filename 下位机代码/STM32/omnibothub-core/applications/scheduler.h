#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "stm32f4xx.h"

typedef struct
{
	u8 check_flag;
	long err_flag;
	long err_flag_1s;
	float err_flag_rate;
	s16 cnt_1ms;
	s16 cnt_2ms;
	s16 cnt_5ms;
	s16 cnt_10ms;
	s16 cnt_20ms;
	s16 cnt_50ms;
	s16 cnt_1s;
	s16 cnt_2s;
	u16 time;
}loop_t;
extern loop_t loop;
typedef struct
{
	float can_task;
	float posforce_task;
	float robot_est_task;
	float ahrs_task;
	float vmc_task;
	float link_task;
	float nav_task;
	float system_task;
}_SYSTEM_DT;

extern _SYSTEM_DT system_dt;

void Loop_check(void);

void Duty_Loop(void);

void Inner_Loop(float);

void Outer_Loop(float);


void Duty_Servo();
void Duty_PForce_Control();
void Duty_Att_Fushion();
void Duty_Link();
void Duty_Navigation();
void Duty_System();
void Duty_Kin_Cal();

extern float trig_test_dt[3];
#endif

