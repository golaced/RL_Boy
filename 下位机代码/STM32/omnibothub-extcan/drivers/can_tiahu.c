#include "can_taihu.h"
#include "include.h"

uint8_t Can_RxData[10]; // receive buffer
uint8_t Can_TxData[10]; // transmit buffer
Ti5_motor_t Ti5_motor={
	. ID=1,
	. mode=9,
	. value=0,
	.pt={
		.kp=2.5f,
	 .targetPosition=-1,
	. kd=0,
	 .targetSpeed=0,
	. targetTorque=1,
		.para={
			.ratio =def_ratio,
	. kt=def_KT,
	. Tur_min=T_MINX,
	. Tur_max=T_MAXX,
	. i_min=I_MINX,
	. i_max=I_MAXX,
		}
	}
};

int ti5_float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float ti5_uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

u8 set_motor_mode_stop(uint16_t id)
{
	CanTxMsg TxMessage;
	TxMessage.IDE = CAN_ID_STD; // CAN_ID_EXT

	TxMessage.RTR = CAN_RTR_DATA;
	
	TxMessage.StdId = id;
	TxMessage.DLC = 1;
	TxMessage.Data[0] = 0X02;
	if(id<7)
		CAN_Transmit(CAN1, &TxMessage);
	else
		CAN_Transmit(CAN2, &TxMessage);
}

u8 set_motor_mode_TargetTorque(uint16_t id, int32_t Val)
{
	CanTxMsg TxMessage;
	TxMessage.IDE = CAN_ID_STD; // CAN_ID_EXT

	TxMessage.RTR = CAN_RTR_DATA;
	
	TxMessage.StdId = id;
	TxMessage.DLC = 5;
	TxMessage.Data[0] = 0X1C;
	TxMessage.Data[1] = (Val) & 0XFF;
	TxMessage.Data[2] = (Val >> 8) & 0XFF;
	TxMessage.Data[3] = (Val >> 16) & 0XFF;
	TxMessage.Data[4] = (Val >> 24) & 0XFF;

	if(id<7)
		CAN_Transmit(CAN1, &TxMessage);
	else
		CAN_Transmit(CAN2, &TxMessage);
}
u8 set_motor_mode_TargetSpeed(uint16_t id, int32_t Val)
{
	CanTxMsg TxMessage;
	TxMessage.IDE = CAN_ID_STD; // CAN_ID_EXT

	TxMessage.RTR = CAN_RTR_DATA;
	
	TxMessage.StdId = id;
	TxMessage.DLC = 5;
	TxMessage.Data[0] = 0X1D;
	TxMessage.Data[1] = (Val) & 0XFF;
	TxMessage.Data[2] = (Val >> 8) & 0XFF;
	TxMessage.Data[3] = (Val >> 16) & 0XFF;
	TxMessage.Data[4] = (Val >> 24) & 0XFF;

	if(id<7)
		CAN_Transmit(CAN1, &TxMessage);
	else
		CAN_Transmit(CAN2, &TxMessage);
}

u8 set_motor_mode_TargetPos(uint16_t id, float Val)
{
	CanTxMsg TxMessage;
	TxMessage.IDE = CAN_ID_STD; // CAN_ID_EXT
	TxMessage.RTR = CAN_RTR_DATA;

	TxMessage.StdId = id;
	TxMessage.DLC = 5;
	
	int32_t  pos_cnt;
//	Val = (Val / 360) * 81 * 65536; // (??????????/360)*?????*65536
	pos_cnt = Val*262144/360;
	TxMessage.Data[0] = 0X1E;
	TxMessage.Data[1] = (pos_cnt) & 0XFF;
	TxMessage.Data[2] = (pos_cnt >> 8) & 0XFF;
	TxMessage.Data[3] = (pos_cnt >> 16) & 0XFF;
	TxMessage.Data[4] = (pos_cnt >> 24) & 0XFF;
	if(id<7)
		CAN_Transmit(CAN1, &TxMessage);
	else
		CAN_Transmit(CAN2, &TxMessage);
}

void send_motor_ctrl_cmd_Ti5(uint16_t motor_id, float kp, float kd, float pos, float spd, float tor)
{
	u8 mbox;
	u16 i = 0;
	int kp_int;
	int kd_int;
	int pos_int;
	int spd_int;
	int tor_int;

	CanTxMsg TxMessage;
	TxMessage.IDE = CAN_ID_STD; // CAN_ID_EXT
	TxMessage.RTR = CAN_RTR_DATA;

	TxMessage.StdId = motor_id;
	TxMessage.DLC = 8;
	if (kp > KP_MAXX)
		kp = KP_MAXX;
	else if (kp < KP_MINX)
		kp = KP_MINX;
	if (kd > KD_MAXX)
		kd = KD_MAXX;
	else if (kd < KD_MINX)
		kd = KD_MINX;
	if (pos > POS_MAXX)
		pos = POS_MAXX;
	else if (pos < POS_MINX)
		pos = POS_MINX;
	if (spd > SPD_MAXX)
		spd = SPD_MAXX;
	else if (spd < SPD_MINX)
		spd = SPD_MINX;
	if (tor > T_MAXX)
		tor = T_MAXX;
	else if (tor < T_MINX)
		tor = T_MINX;

	kp_int = ti5_float_to_uint(kp, KP_MINX, KP_MAXX, 12);
	kd_int = ti5_float_to_uint(kd, KD_MINX, KD_MAXX, 9);
	pos_int = ti5_float_to_uint(pos, POS_MINX, POS_MAXX, 16);
	spd_int = ti5_float_to_uint(spd, SPD_MINX, SPD_MAXX, 12);
	tor_int = ti5_float_to_uint(tor, T_MINX, T_MAXX, 12);

	TxMessage.Data[0] = 0x00 | (kp_int >> 7);
	TxMessage.Data[1] = ((kp_int & 0X7F) << 1) | ((kd_int & 0X100) >> 8);
	TxMessage.Data[2] = kd_int & 0XFF;
	TxMessage.Data[3] = pos_int >> 8;
	TxMessage.Data[4] = pos_int & 0XFF;
	TxMessage.Data[5] = spd_int >> 4;
	TxMessage.Data[6] = ((spd_int & 0X0F) << 4) | (tor_int >> 8);
	TxMessage.Data[7] = tor_int & 0XFF;
	
	if(motor_id<7)
		CAN_Transmit(CAN1, &TxMessage);
	else
		CAN_Transmit(CAN2, &TxMessage);
}

void SET_PT_Para_DLC6(uint8_t id,uint8_t cmd,uint8_t RW,int32_t value)
{
	CanTxMsg TxMessage;
	TxMessage.IDE = CAN_ID_STD; // CAN_ID_EXT
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 6;
	
	TxMessage.StdId = id;
	TxMessage.Data[0] = cmd;
	TxMessage.Data[1] = RW;
	TxMessage.Data[2] = (value) & 0XFF;
	TxMessage.Data[3] = (value >> 8) & 0XFF;
	TxMessage.Data[4] = (value >> 16) & 0XFF;
	TxMessage.Data[5] = (value >> 24) & 0XFF;
	if(id<7)
		CAN_Transmit(CAN1, &TxMessage);
	else
		CAN_Transmit(CAN2, &TxMessage);
}

void SendFloatAsInt32(uint8_t id, uint8_t cmd, uint8_t RW, float floatValue) {
    int32_t intValue;

	if(RW == DLC_6_Write)
	{   
		memcpy(&intValue, &floatValue, sizeof(float));
//		int32_t *wfValue = (int32_t *)&floatValue; // float????
//     intValue = (int32_t)*wfValue;
	}
	else	if(RW == DLC_6_Read)
	{   
		intValue =0;
	}
    SET_PT_Para_DLC6(id, cmd, RW, intValue);
}

void SET_SAVE_DLC2(uint8_t id)
{
	CanTxMsg TxMessage;
	TxMessage.IDE = CAN_ID_STD; // CAN_ID_EXT
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 1;
	
	TxMessage.StdId = id;
	TxMessage.Data[0] = 0X0E;
	if(id<7)
		CAN_Transmit(CAN1, &TxMessage);
	else
		CAN_Transmit(CAN2, &TxMessage);
}

void Motor_Init(uint8_t id)
{
	CanTxMsg TxMessage;

}

void mit_bldc_thread_taihu(char en,float dt){

	int i;
	float set_q;
	for(i=0;i<14;i++)
	{
		motor_chassis[i].param.id=i;
		motor_chassis[i].param.connect=1;
		set_q=motor_chassis[i].param.set_q=motor_chassis[i].set_q+motor_chassis[i].set_q_test_bias;
		set_motor_mode_TargetPos(motor_chassis[i].param.id,set_q);
		send_motor_ctrl_cmd_Ti5(Ti5_motor.ID,
		2.5,
		0.5,
		set_q,
		0,
		0);
	}
	#if 0
	switch(Ti5_motor.mode)
	{
		case 0:{
			set_motor_mode_stop(Ti5_motor.ID);
		}break;
		case 1:{
				set_motor_mode_TargetTorque(Ti5_motor.ID,Ti5_motor.value);
		}break;
		case 2:{
				set_motor_mode_TargetSpeed(Ti5_motor.ID,Ti5_motor.value);
		}break;
		case 3:{
				set_motor_mode_TargetPos(Ti5_motor.ID,Ti5_motor.value);
		}break;
		case 4:{	
				send_motor_ctrl_cmd_Ti5(Ti5_motor.ID,
			Ti5_motor.pt.kp,
			Ti5_motor.pt.kd,
			Ti5_motor.pt.targetPosition,
			Ti5_motor.pt.targetSpeed,
			Ti5_motor.pt.targetTorque);
		}break;
				case 9:{	

		}break;
		//??
				case 90:{
			Ti5_motor.mode =91;
			SendFloatAsInt32(Ti5_motor.ID,DLC_6_MOTOR_RATIO,DLC_6_Read,Ti5_motor.pt.para.ratio);
		}break;		
		case 91:{
				Ti5_motor.mode =92;
			SendFloatAsInt32(Ti5_motor.ID,DLC_6_POS_TORQUE_KT,DLC_6_Read,Ti5_motor.pt.para.kt);
		}break;			
		case 92:{Ti5_motor.mode =93;
			SendFloatAsInt32(Ti5_motor.ID,DLC_6_POS_TORQUE_T_MINX,DLC_6_Read,Ti5_motor.pt.para.Tur_min);
		}break;			
		case 93:{Ti5_motor.mode =94;
			SendFloatAsInt32(Ti5_motor.ID,DLC_6_POS_TORQUE_T_MAXX,DLC_6_Read,Ti5_motor.pt.para.Tur_max);
		}break;			
		case 94:{Ti5_motor.mode =95;
			SendFloatAsInt32(Ti5_motor.ID,DLC_6_POS_TORQUE_I_MINX,DLC_6_Read,Ti5_motor.pt.para.i_min);
		}break;	
				case 95:{Ti5_motor.mode =9;
			SendFloatAsInt32(Ti5_motor.ID,DLC_6_POS_TORQUE_I_MAXX,DLC_6_Read,Ti5_motor.pt.para.i_max);
		}break;
		///??
		case 100:{
			Ti5_motor.mode =101;
			SET_PT_Para_DLC6(Ti5_motor.ID,DLC_6_MOTOR_RATIO,DLC_6_Write,Ti5_motor.pt.para.ratio);
		}break;		
		case 101:{
				Ti5_motor.mode =102;
			SendFloatAsInt32(Ti5_motor.ID,DLC_6_POS_TORQUE_KT,DLC_6_Write,Ti5_motor.pt.para.kt);
		}break;			
		case 102:{Ti5_motor.mode =103;
			SendFloatAsInt32(Ti5_motor.ID,DLC_6_POS_TORQUE_T_MINX,DLC_6_Write,Ti5_motor.pt.para.Tur_min);
		}break;			
		case 103:{Ti5_motor.mode =104;
			SendFloatAsInt32(Ti5_motor.ID,DLC_6_POS_TORQUE_T_MAXX,DLC_6_Write,Ti5_motor.pt.para.Tur_max);
		}break;			
		case 104:{Ti5_motor.mode =105;
			SendFloatAsInt32(Ti5_motor.ID,DLC_6_POS_TORQUE_I_MINX,DLC_6_Write,Ti5_motor.pt.para.i_min);
		}break;	
			case 105:{Ti5_motor.mode =106;
			SendFloatAsInt32(Ti5_motor.ID,DLC_6_POS_TORQUE_I_MAXX,DLC_6_Write,Ti5_motor.pt.para.i_max);
		}break;	
			case 106:{Ti5_motor.mode =9;
			SET_SAVE_DLC2(Ti5_motor.ID);
		}break;
	}
	#endif
}
