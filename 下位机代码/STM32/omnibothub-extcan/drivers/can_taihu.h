#ifndef __CAN_RV_TI5_H
#define __CAN_RV_TI5_H
#include "can.h"
#include "delay.h"
typedef struct
{
	int32_t ratio;
	float kt;
	float Tur_min;
	float Tur_max;
	float i_min;
	float i_max;
}para_t;
typedef struct
{
	float kp;
	float targetPosition;//
	float kd;
	float targetSpeed;//
	float targetTorque;//
	float pos_rad;//
	float speed_rad;//
	float current;//
	float temp1;//
	float temp2;//
	para_t para;
}ptmode_t;

typedef struct
{
	uint8_t ID;
	uint8_t mode;	
	int32_t value;  
	ptmode_t pt;
}Ti5_motor_t;
extern Ti5_motor_t Ti5_motor;
static float timer_ti5=0;
#define Ti5motor_5060 5060
#define Ti5motor_6070 6070
#define Ti5motor_7080 6080

#define ACTUATOR_TYPE Ti5motor_5060

#if (ACTUATOR_TYPE == Ti5motor_5060)
#define def_ratio 51
#define def_KT 0.089f
#define T_MINX -6.6f * 2
#define T_MAXX 6.6f * 2
#define I_MINX -9.0f
#define I_MAXX 9.0f
#elif (ACTUATOR_TYPE == Ti5motor_6070)
#define def_ratio 51
#define def_KT 0.096f
#define T_MINX -19.8f * 2 
#define T_MAXX 19.8f * 2
#define I_MINX -20.0f
#define I_MAXX 20.0f
#elif (ACTUATOR_TYPE == Ti5motor_7080)
#define def_ratio 51
#define def_KT 0.118f
#define T_MINX -69.0f
#define T_MAXX 69.0f
#define I_MINX -8.4f 
#define I_MAXX 8.4f
#endif
#define KP_MINX 0.0f
#define KP_MAXX 500.0f
#define KD_MINX 0.0f
#define KD_MAXX 5.0f
#define POS_MINX -12.5f
#define POS_MAXX 12.5f
#define SPD_MINX -18.0f
#define SPD_MAXX 18.0f
u8 set_motor_mode_TargetTorque( uint16_t id,int32_t Val);
u8 set_motor_mode_TargetSpeed( uint16_t id,int32_t Val);
u8 set_motor_mode_TargetPos( uint16_t id,float Val);
void send_motor_ctrl_cmd_Ti5(uint16_t motor_id,float kp,float kd,float pos,float spd,float tor);
u8 set_motor_mode_stop( uint16_t id);

#define DLC_6_Read 0x40	
#define DLC_6_Write 0x20	
#define DLC_6_MOTOR_RATIO 0x41
#define DLC_6_POS_TORQUE_KP 0x42
#define DLC_6_POS_TORQUE_KD 0x43
#define DLC_6_POS_TORQUE_KT 0x44
#define DLC_6_POS_TORQUE_KV 0x45
#define DLC_6_POS_TORQUE_T_MINX 0x46
#define DLC_6_POS_TORQUE_T_MAXX 0x47
#define DLC_6_POS_TORQUE_I_MINX 0x48
#define DLC_6_POS_TORQUE_I_MAXX 0x49
extern void SendFloatAsInt32(uint8_t id, uint8_t cmd, uint8_t RW, float floatValue);
extern void SET_SAVE_DLC2(uint8_t id);

void mit_bldc_thread_taihu(char en,float dt);
#endif
