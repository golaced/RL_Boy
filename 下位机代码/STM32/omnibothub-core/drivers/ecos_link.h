#ifndef __CAN_RV_H
#define __CAN_RV_H
#include "can.h"
#include "delay.h"

#define param_get_pos	0x01
#define param_get_spd	0x02
#define param_get_cur	0x03
#define param_get_pwr	0x04
#define param_get_acc	0x05
#define param_get_lkgKP	0x06
#define param_get_spdKI	0x07
#define param_get_fdbKP	0x08
#define param_get_fdbKD	0x09

#define comm_ack	0x00
#define comm_auto	0x01

typedef struct 
{
	uint16_t motor_id;
	uint8_t INS_code;		//instruction code.
	uint8_t motor_fbd;	//motor CAN communication feedback.
}MotorCommFbd;

typedef struct 
{
	uint16_t angle_actual_int;
	uint16_t angle_desired_int;
	int16_t speed_actual_int;
	int16_t speed_desired_int;
	int16_t current_actual_int;
	int16_t current_desired_int;
	float 	speed_actual_rad;
	float 	speed_desired_rad;
	float 	angle_actual_rad;	
	float   angle_desired_rad;
	uint16_t	motor_id;
	uint8_t 	temperature;
	uint8_t		error;
	float     angle_actual_float;
	float 		speed_actual_float;
	float 		current_actual_float;
	float     angle_desired_float;
	float 		speed_desired_float;
	float 		current_desired_float;
	float			power;
	uint16_t	acceleration;
	uint16_t	linkage_KP;
	uint16_t 	speed_KI;
	uint16_t	feedback_KP;
	uint16_t	feedback_KD;
}OD_Motor_Msg;

extern OD_Motor_Msg rv_motor_msg[14];
extern uint16_t motor_id_check;

void MotorIDReset(void);
void MotorIDSetting(uint16_t motor_id,uint16_t motor_id_new);
void MotorSetting(uint16_t motor_id,uint8_t cmd);
void MotorCommModeReading(uint16_t motor_id);
void MotorIDReading(void);

void send_motor_ctrl_cmd(uint16_t motor_id,float kp,float kd,float pos,float spd,float cur);
void set_motor_position(uint16_t motor_id,float pos,uint16_t spd,uint16_t cur,uint8_t ack_status);
void set_motor_speed(uint16_t motor_id,float spd,uint16_t cur,uint8_t ack_status);
void set_motor_cur_tor(uint16_t motor_id,int16_t cur_tor,uint8_t ctrl_status,uint8_t ack_status);
void set_motor_acceleration(uint16_t motor_id,uint16_t acc,uint8_t ack_status);	
void set_motor_linkage_speedKI(uint16_t motor_id,uint16_t linkage,uint16_t speedKI,uint8_t ack_status);
void set_motor_feedbackKP(uint16_t motor_id,uint16_t fdbKP,uint8_t ack_status);
void get_motor_parameter(uint16_t motor_id,uint8_t param_cmd);

void set_motors_current(OD_Motor_Msg rv_MotorMsg[8],uint8_t motor_quantity);
void RV_can_data_repack(CanRxMsg *RxMessage,uint8_t comm_mode,uint8_t sel);

int float_to_uint(float x, float x_min, float x_max, int bits);

void mit_bldc_thread_rv(char en_all,float dt);
u8 mit_rv_pos_zero( char id);

#endif
