#include "can.h"
#include "include.h"
#include "base_struct.h"
#include "math.h"
#include "can.h"
#include "gait_math.h"
char state_mit[4]={0};

float P_MIN_CAN_MIT[14] ={-12.5f,-12.5f,-12.5f};
float P_MAX_CAN_MIT[14]  ={12.5f,12.5f,12.5f};
float V_MIN_CAN_MIT[14] ={-38.2f,-38.2f,-38.2f};
float V_MAX_CAN_MIT[14] ={38.2f,38.2f,38.2f};
float KP_MIN_CAN_MIT[14] ={0.0f,0.0f,0.0f};
float KP_MAX_CAN_MIT[14] ={500.0f,500.0f,500.0f};
float KD_MIN_CAN_MIT[14] ={0.0f,0.0f,0.0f};
float KD_MAX_CAN_MIT[14] ={5.0f,5.0f,5.0f};
float T_MIN_CAN_MIT[14] ={-12.0f,-12.0f,-12.0f};
float T_MAX_CAN_MIT[14] ={12.0f,12.0f,12.0f};

float fmaxf_mit(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }

float fminf_mit(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }

int float_to_uint_mit(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
		x=LIMIT(x,x_min,x_max);
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }

float uint_to_float_mit(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

uint16_t float_to_uint_gim(float v,float v_min,float v_max,uint32_t width)
{
	float temp;
	int32_t utemp;
	temp=((v-v_min)/(v_max-v_min))*((float)width);
	utemp=(int32_t)temp;
	if(utemp<0)utemp=0;
	if(utemp>width)utemp=width;
	return utemp;
}
		
u8 mit_motor_mode_en( char id,char en){//使能电机
	u8 mbox;
  u16 i=0;
	CanTxMsg TxMessage;
	if(id<7)
		TxMessage.StdId=0x00+id+1;	 // 标准标识符为0
	else
		TxMessage.StdId=0x00+id+1-7;
	
  TxMessage.ExtId=0x00;//0x200;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;							 // 发送两帧信息
	TxMessage.Data[0] = 0xFF;//300??
	TxMessage.Data[1] = 0xFF;
	TxMessage.Data[2] = 0xFF;
	TxMessage.Data[3] = 0xFF;
	TxMessage.Data[4] = 0xFF;
	TxMessage.Data[5] = 0xFF;
	TxMessage.Data[6] = 0xFF;
	if(en)
		TxMessage.Data[7] = 0xFC;
	else
		TxMessage.Data[7] = 0xFD;
	
	if(id<7){
		mbox= CAN_Transmit(CAN1, &TxMessage);   
		i=0;
		while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	}
	else{
		mbox= CAN_Transmit(CAN2, &TxMessage);   
		i=0;
		while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	}
  if(i>=0XFFF)return 1;
  return 0;		
}	

u8 mit_set_pos_zero( char id){//设置零位
	u8 mbox;
  u16 i=0;
	CanTxMsg TxMessage;
	if(id<7)
		TxMessage.StdId=0x00+id+1;	 // 标准标识符为0
	else
		TxMessage.StdId=0x00+id+1-7;
	
  TxMessage.ExtId=0x00;//0x200;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;							 // 发送两帧信息
	TxMessage.Data[0] = 0xFF;//300??
	TxMessage.Data[1] = 0xFF;
	TxMessage.Data[2] = 0xFF;
	TxMessage.Data[3] = 0xFF;
	TxMessage.Data[4] = 0xFF;
	TxMessage.Data[5] = 0xFF;
	TxMessage.Data[6] = 0xFF;
	TxMessage.Data[7] = 0xFE;
	
	if(id<7){
		mbox= CAN_Transmit(CAN1, &TxMessage);   
		i=0;
		while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	}
	else{
		mbox= CAN_Transmit(CAN2, &TxMessage);   
		i=0;
		while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	}
  if(i>=0XFFF)return 1;
  return 0;		
}	

int qd_mid_f_mit=3;
uint16_t int_temp;
float over_reg1[1000]={0};
float over_reg2[1000]={0};
float over_reg3[1000]={0};
int over_cnt1=0;
int over_cnt2=0;
int over_cnt3=0;
int en_over_save1=0;
int en_over_save2=0;
int en_over_save3=0;
float t_check_over=12;
float err_dead=1.2;
void data_can_mit_anal(motor_measure_t *ptr,uint8_t buf_rx[8])//解析电机数据
{
	char i;	
	char sum = 0;
	float dt= Get_Cycle_T(ptr->param.id+50); 	
	float t_temp=0;
	uint16_t _cnt=0,cnt_reg;

	int inv_q_flag=1;
	if(!ptr->param.q_flag)
		inv_q_flag=-1;

	ptr->param.connect=1;
	ptr->param.loss_cnt=0;
	ptr->param.rx_dt=dt;

	uint16_t p_int = (buf_rx[1]<<8)|buf_rx[2];
	uint16_t v_int = (buf_rx[3]<<4)|(buf_rx[4]>>4);
	uint16_t i_int = ((buf_rx[4]&0xF)<<8)|buf_rx[5];
  
	p_int = (buf_rx[1]<<8)|buf_rx[2];
	v_int = (buf_rx[3]<<4)|(buf_rx[4]>>4);
	i_int = ((buf_rx[4]&0xF)<<8)|buf_rx[5];
	ptr->param.total_angle_out=	 uint_to_float_mit(p_int, P_MIN_CAN_MIT[ptr->param.id], P_MAX_CAN_MIT[ptr->param.id], 16)*57.3;
	ptr->t_now_flt= uint_to_float_mit(i_int, -T_MAX_CAN_MIT[ptr->param.id], T_MAX_CAN_MIT[ptr->param.id], 12)*ptr->param.t_inv_flag_measure*inv_q_flag;

	ptr->param.cnt_rotate=(int)(ptr->param.total_angle_out)/180;
	float total_angle_out_single=fmod(
	(float)(ptr->param.total_angle_out),360);
	ptr->param.total_angle_out_single=total_angle_out_single;

  ptr->q_now=inv_q_flag*To_180_degrees(ptr->param.total_angle_out_single);
	ptr->q_now_flt=To_180_degrees(ptr->q_now+ptr->param.q_reset_angle);

	ptr->qd_now=To_180_degrees(ptr->q_now-ptr->param.q_now_reg)/dt;//角速度微分  使用TD？
	ptr->qd_now_flt=Moving_Median(ptr->param.id,qd_mid_f_mit,ptr->qd_now);	

	ptr->param.q_now_reg=ptr->q_now;
	ptr->param.qd_now_reg=ptr->qd_now;
}

float v_des_ff[14];
float t_des_ff[14];
float k_spd_all=5;//butler new
//float k_spd_all=1;//butler old
char data_can_mit_send(motor_measure_t *ptr){//发送控制指令
	u8 canbuft_mit1[20]={0};
	int q_flag=1;
	float set_q;
	
	if(ptr->param.q_flag)
		q_flag=1;
	else
		q_flag=-1;
	if(ptr->cmd_mode==2)
		ptr->set_q=LIMIT(ptr->set_q_test+ptr->set_q_test_bias,-180,180);
	
	#if EN_MIT_PID_INNER
		float temp=0;
		if(ptr->param.q_reset_angle==180){
			if(ptr->set_q>=-180&&ptr->set_q<=-0){
				if(ptr->param.q_flag)
					temp= 180-fabs(ptr->set_q);
				else
					temp= -(180-fabs(ptr->set_q));
			}else if(ptr->set_q<=180&&ptr->set_q>=0){
				if(!ptr->param.q_flag)
					temp= 180-fabs(ptr->set_q);
				else
					temp= -(180-fabs(ptr->set_q));
			}
		}else{//------------------------------------- 
			temp=q_flag*(ptr->set_q-ptr->param.q_reset_angle);
		}
		set_q=temp;
	#endif

	ptr->param.set_q=set_q;//for record

	float set_dq=q_flag*ptr->set_qd/57.3;
	float set_t=q_flag*ptr->set_t;
		
	float p_des = fminf_mit(fmaxf_mit(P_MIN_CAN_MIT[ptr->param.id], set_q/57.3), P_MAX_CAN_MIT[ptr->param.id]);        
	float v_des = fminf_mit(fmaxf_mit(V_MIN_CAN_MIT[ptr->param.id], set_dq), V_MAX_CAN_MIT[ptr->param.id]);  
	float kp = fminf_mit(fmaxf_mit(KP_MIN_CAN_MIT[ptr->param.id], ptr->stiff*ptr->kp*EN_MIT_PID_INNER), KP_MAX_CAN_MIT[ptr->param.id]);   
	float kd = fminf_mit(fmaxf_mit(KD_MIN_CAN_MIT[ptr->param.id], ptr->stiff*ptr->kd*EN_MIT_PID_INNER), KD_MAX_CAN_MIT[ptr->param.id]); 
	float t_ff = fminf_mit(fmaxf_mit(T_MIN_CAN_MIT[ptr->param.id],LIMIT(set_t,-ptr->max_t,ptr->max_t)), T_MAX_CAN_MIT[ptr->param.id]); //最终发送的力矩前馈
	t_des_ff[ptr->param.id]=t_ff;
	if(EN_MIT_PID_INNER==0||ptr->param.usb_cmd_mode==2)//纯力矩前馈、电流模式
	   kp=kd=v_des=0;
	
	if(ptr->param.control_mode==1){//转速模式7
		kp=0;
		kd=LIMIT(kd,KD_MIN_CAN_MIT[ptr->param.id],KD_MAX_CAN_MIT[ptr->param.id]);
		v_des_ff[ptr->param.id]=v_des=LIMIT(set_dq*k_spd_all,V_MIN_CAN_MIT[ptr->param.id],V_MAX_CAN_MIT[ptr->param.id]);//butler
	}
			
	/// convert floats to unsigned ints ///    
	uint16_t p_int = float_to_uint_mit(p_des, P_MIN_CAN_MIT[ptr->param.id], P_MAX_CAN_MIT[ptr->param.id], 16);      
	uint16_t v_int = float_to_uint_mit(v_des, V_MIN_CAN_MIT[ptr->param.id], V_MAX_CAN_MIT[ptr->param.id], 12);    
	uint16_t kp_int = float_to_uint_mit(kp, KP_MIN_CAN_MIT[ptr->param.id], KP_MAX_CAN_MIT[ptr->param.id], 12);    
	uint16_t kd_int = float_to_uint_mit(kd, KD_MIN_CAN_MIT[ptr->param.id], KD_MAX_CAN_MIT[ptr->param.id], 12);    
	uint16_t t_int = float_to_uint_mit(t_ff, T_MIN_CAN_MIT[ptr->param.id], T_MAX_CAN_MIT[ptr->param.id], 12);  

	canbuft_mit1[0] = p_int>>8;      
	canbuft_mit1[1] = p_int&0xFF;  
	canbuft_mit1[2] = v_int>>4;      
	canbuft_mit1[3] = ((v_int&0xF)<<4)|(kp_int>>8);   
	canbuft_mit1[4] = kp_int&0xFF;  
	canbuft_mit1[5] = kd_int>>4;    
	canbuft_mit1[6] = ((kd_int&0xF)<<4)|(t_int>>8);   
	canbuft_mit1[7] = t_int&0xff;   

	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage1;
	if(ptr->param.id<7)
		TxMessage1.StdId=0x00+ptr->param.id+1;	 // 标准标识符为0
	else
		TxMessage1.StdId=0x00+ptr->param.id+1-7;
	
	TxMessage1.ExtId=0x00;//0x200;	 // 设置扩展标示符（29位）
	TxMessage1.IDE=0;		  // 使用扩展标识符
	TxMessage1.RTR=0;		  // 消息类型为数据帧，一帧8位
	TxMessage1.DLC=8;							 // 发送两帧信息
	TxMessage1.Data[0] = canbuft_mit1[0];//300??
	TxMessage1.Data[1] = canbuft_mit1[1];
	TxMessage1.Data[2] = canbuft_mit1[2];
	TxMessage1.Data[3] = canbuft_mit1[3];
	TxMessage1.Data[4] = canbuft_mit1[4];
	TxMessage1.Data[5] = canbuft_mit1[5];
	TxMessage1.Data[6] = canbuft_mit1[6];
	TxMessage1.Data[7] = canbuft_mit1[7];
	if(ptr->param.id==1)
		i=0;
	#if 1
		if(ptr->param.id<7){
			mbox= CAN_Transmit(CAN1, &TxMessage1);   
			i=0;
			while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
		}
		else{
			mbox= CAN_Transmit(CAN2, &TxMessage1);   
			i=0;
			while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
		}
		if(i>=0XFFF)return 1;
	#endif
  return 0;		
}

char data_can_sample_only(motor_measure_t *ptr){//数据采集
	u8 canbuft1[8];
/// limit data to be within bounds ///  
	int q_flag=1;
	if(ptr->param.q_flag)
		q_flag=1;
	else
		q_flag=-1; 
	float set_q=q_flag*To_180_degrees(ptr->set_q-To_180_degrees(ptr->param.q_reset_angle));
	ptr->param.set_q=set_q;
 
	float p_des = fminf_mit(fmaxf_mit(P_MIN_CAN_MIT[ptr->param.id], set_q/57.3), P_MAX_CAN_MIT[ptr->param.id]);        
	float v_des = fminf_mit(fmaxf_mit(V_MIN_CAN_MIT[ptr->param.id], 0/57.3), V_MAX_CAN_MIT[ptr->param.id]);   
	float kp = fminf_mit(fmaxf_mit(KP_MIN_CAN_MIT[ptr->param.id],  0), KP_MAX_CAN_MIT[ptr->param.id]);   
	float kd = fminf_mit(fmaxf_mit(KD_MIN_CAN_MIT[ptr->param.id],  0), KD_MAX_CAN_MIT[ptr->param.id]); 
	float t_ff = fminf_mit(fmaxf_mit(T_MIN_CAN_MIT[ptr->param.id], 0), T_MAX_CAN_MIT[ptr->param.id]); 
	/// convert floats to unsigned ints ///    
	uint16_t p_int = float_to_uint_mit(p_des, P_MIN_CAN_MIT[ptr->param.id], P_MAX_CAN_MIT[ptr->param.id], 16);      
	uint16_t v_int = float_to_uint_mit(v_des, V_MIN_CAN_MIT[ptr->param.id], V_MAX_CAN_MIT[ptr->param.id], 12);    
	uint16_t kp_int = float_to_uint_mit(kp, KP_MIN_CAN_MIT[ptr->param.id], KP_MAX_CAN_MIT[ptr->param.id], 12);    
	uint16_t kd_int = float_to_uint_mit(kd, KD_MIN_CAN_MIT[ptr->param.id], KD_MAX_CAN_MIT[ptr->param.id], 12);    
	uint16_t t_int = float_to_uint_mit(t_ff, T_MIN_CAN_MIT[ptr->param.id], T_MAX_CAN_MIT[ptr->param.id], 12);  

	canbuft1[0] = p_int>>8;      
	canbuft1[1] = p_int&0xFF;  
	canbuft1[2] = v_int>>4;      
	canbuft1[3] = ((v_int&0xF)<<4)|(kp_int>>8);   
	canbuft1[4] = kp_int&0xFF;  
	canbuft1[5] = kd_int>>4;    
	canbuft1[6] = ((kd_int&0xF)<<4)|(t_int>>8);   
	canbuft1[7] = t_int&0xff;   
	
	u8 mbox;
  u16 i=0;
	CanTxMsg TxMessage1;
	if(ptr->param.id<7)
		TxMessage1.StdId=0x00+ptr->param.id+1;	 // 标准标识符为0
	else
		TxMessage1.StdId=0x00+ptr->param.id+1-7;
	
  TxMessage1.ExtId=0x00;//0x200;	 // 设置扩展标示符（29位）
  TxMessage1.IDE=0;		  // 使用扩展标识符
  TxMessage1.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage1.DLC=8;							 // 发送两帧信息
	TxMessage1.Data[0] = canbuft1[0];//300??
	TxMessage1.Data[1] = canbuft1[1];
	TxMessage1.Data[2] = canbuft1[2];
	TxMessage1.Data[3] = canbuft1[3];
	TxMessage1.Data[4] = canbuft1[4];
	TxMessage1.Data[5] = canbuft1[5];
	TxMessage1.Data[6] = canbuft1[6];
	TxMessage1.Data[7] = canbuft1[7];
	
	if(ptr->param.id<7){
		mbox= CAN_Transmit(CAN1, &TxMessage1);   
		i=0;
		while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	}
	else{
		mbox= CAN_Transmit(CAN2, &TxMessage1);   
		i=0;
		while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	}
  if(i>=0XFFF)return 1;
  return 0;		
}

char en_test=0;
char en_mit_out=0;
int mit_delay_d[2]={100,200};
int mit_delay=200;//300
int mit_connect_cnt=0;
float test_cmd[2]={3.5,25};
void mit_bldc_thread(char en_all,float dt)
{
	char i=0;
	static char state_reg[4]={0};
	static float auto_off_t=99;
	static float timer_sin=0;
	static int reg_cmd_mode;
	#if 0//Tinker
		motor_chassis[0].motor.type=DM_6006; motor_chassis[0].param.q_flag=0;
		motor_chassis[1].motor.type=DM_8006; motor_chassis[1].param.q_flag=1;
		motor_chassis[2].motor.type=DM_8006; motor_chassis[2].param.q_flag=1;
		motor_chassis[3].motor.type=DM_8006; motor_chassis[3].param.q_flag=0;
		motor_chassis[4].motor.type=DM_6006; motor_chassis[4].param.q_flag=1;
		
		motor_chassis[6].motor.type=DM_6006; motor_chassis[6].param.q_flag=0;
		motor_chassis[7].motor.type=DM_8006; motor_chassis[7].param.q_flag=1;
		motor_chassis[8].motor.type=DM_8006; motor_chassis[8].param.q_flag=0;
		motor_chassis[9].motor.type=DM_8006; motor_chassis[9].param.q_flag=1;
		motor_chassis[10].motor.type=DM_6006;motor_chassis[10].param.q_flag=0;
		
		for(i=0;i<1;i++)
			motor_chassis[i].param.control_mode=0;//POS
	
	#endif
	for(i=0;i<14;i++){
		motor_chassis[i].param.id=i;
		switch(motor_chassis[i].motor.type){
	    case M_3508:		
			P_MIN_CAN_MIT[i] =-12.566f;
			P_MAX_CAN_MIT[i]  =12.566f;
			V_MIN_CAN_MIT[i] =-50.0f;
			V_MAX_CAN_MIT[i] = 50.0f;
			KP_MIN_CAN_MIT[i] =0.0f;
			KP_MAX_CAN_MIT[i] =500.0f;
			KD_MIN_CAN_MIT[i] =0.0f;
			KD_MAX_CAN_MIT[i] =5.0f;

			T_MIN_CAN_MIT[i] =-5.0f;
			T_MAX_CAN_MIT[i] = 5.0f;
			motor_chassis[i].motor.anal_type=M_MIT;
			break;
			case DM_J4310:		
			P_MIN_CAN_MIT[i] =-12.5f;
			P_MAX_CAN_MIT[i]  =12.5f;
			V_MIN_CAN_MIT[i] =-30.0f;
			V_MAX_CAN_MIT[i] = 30.0f;
			KP_MIN_CAN_MIT[i] =0.0f;
			KP_MAX_CAN_MIT[i] =500.0f;
			KD_MIN_CAN_MIT[i] =0.0f;
			KD_MAX_CAN_MIT[i] =5.0f;

			T_MIN_CAN_MIT[i] =-10.0f;
			T_MAX_CAN_MIT[i] = 10.0f;
			motor_chassis[i].motor.anal_type=M_MIT;
			break;
			case DM_6006://小型扁平电机
			P_MIN_CAN_MIT[i] =-12.5f;
			P_MAX_CAN_MIT[i]  =12.5f;
			V_MIN_CAN_MIT[i] =-45.0f;
			V_MAX_CAN_MIT[i] = 45.0f;
			KP_MIN_CAN_MIT[i] =0.0f;
			KP_MAX_CAN_MIT[i] =500.0f;
			KD_MIN_CAN_MIT[i] =0.0f;
			KD_MAX_CAN_MIT[i] =5.0f;

			T_MIN_CAN_MIT[i] =-12.0f;
			T_MAX_CAN_MIT[i] = 12.0f;
			motor_chassis[i].motor.anal_type=M_MIT;
			break;
			case DM_8006 ://减速比6 电机		
			P_MIN_CAN_MIT[i] =-12.5f;
			P_MAX_CAN_MIT[i]  =12.5f;
			V_MIN_CAN_MIT[i] =-45.0f;
			V_MAX_CAN_MIT[i] = 45.0f;
			KP_MIN_CAN_MIT[i] =0.0f;
			KP_MAX_CAN_MIT[i] =500.0f;
			KD_MIN_CAN_MIT[i] =0.0f;
			KD_MAX_CAN_MIT[i] =5.0f;

			T_MIN_CAN_MIT[i] =-20.0f;
			T_MAX_CAN_MIT[i] = 20.0f;
			motor_chassis[i].motor.anal_type=M_MIT;
			break;
			default:		
			P_MIN_CAN_MIT[i] =-12.5f;
			P_MAX_CAN_MIT[i]  =12.5f;
			V_MIN_CAN_MIT[i] =-30.0f;
			V_MAX_CAN_MIT[i] = 30.0f;
			KP_MIN_CAN_MIT[i] =0.0f;
			KP_MAX_CAN_MIT[i] =500.0f;
			KD_MIN_CAN_MIT[i] =0.0f;
			KD_MAX_CAN_MIT[i] =5.0f;

			T_MIN_CAN_MIT[i] =-10.0f;
			T_MAX_CAN_MIT[i] = 10.0f;
			motor_chassis[i].motor.anal_type=M_MIT;
			break;
			}
	}
	//test
	timer_sin+=dt*test_cmd[0];
	mit_connect_cnt=0;
	for(i=0;i<14;i++){
		if(motor_chassis[i].param.connect)
			mit_connect_cnt++;
		if(reg_cmd_mode!=motor_chassis[0].cmd_mode||!en_all){
			motor_chassis[i].set_q_test_bias=motor_chassis[i].q_now_flt;
			timer_sin=0;
		}
		if(motor_chassis[i].cmd_mode==2){
			motor_chassis[i].set_q_test=sin(timer_sin)*test_cmd[1];
		}
	}
	reg_cmd_mode=motor_chassis[0].cmd_mode;
	//标定
	for(i=0;i<14;i++){
		if((motor_chassis[i].reset_q==1||motor_chassis[i].cal_div==1)&&motor_chassis[i].reset_q_lock==0){
				mit_set_pos_zero(i);
				delay_us(mit_delay);
				motor_chassis[i].reset_q_lock=1;
        motor_chassis[i].reset_q_cnt=0;
			  motor_chassis[i].reset_q=0;
		}
		if(motor_chassis[i].reset_q_lock==1)
		{
				mit_set_pos_zero(i);
				delay_us(mit_delay);
        motor_chassis[i].reset_q_cnt++;
				if(motor_chassis[i].reset_q_cnt>3)
				{
					motor_chassis[i].reset_q_cnt=0;
					motor_chassis[i].reset_q_lock=2;
				}
		}
		if(motor_chassis[i].reset_q_lock==2)
		{
			 motor_chassis[i].reset_q_delay_timer+=dt;
			 if(motor_chassis[i].reset_q_delay_timer>3.5)
			 {
				 motor_chassis[i].reset_q_delay_timer=0;
				 motor_chassis[i].reset_q_lock=0;
				 motor_chassis[i].reset_q=0;
				 motor_chassis[i].reset_q_cnt=0;
			 }
		}
	}		
	//保存配置
	if(can_write_flash==1){
			//stop motor
			for(i=0;i<14;i++){
				motor_chassis[i].en_cmd=0;
				motor_chassis[i].param.given_current=0;
				motor_chassis[i].given_current_cmd=0;
				motor_chassis[i].set_t=0;	
				motor_chassis[i].motor.ready=0;
			}
			can_write_flash=0;
			en_all=0;
			WRITE_PARM();
  }
			
	#if 1//发送控制指令
		for(i=0;i<14;i++){
			if(en_mit_out==2)//使能发送命令 MIT控制模式
			{
				data_can_mit_send(&motor_chassis[i]);
				delay_us(mit_delay);
			}else//否则就采集数据  电机扭矩均为0
			{
				data_can_sample_only(&motor_chassis[i]);
				delay_us(mit_delay);
			}
		}
	#else
		for(i=0;i<6;i++){
			if(en_mit_out==2)//使能发送命令 MIT控制模式
			{
				data_can_mit_send(&motor_chassis[i]);
				delay_us(mit_delay_d[0]);
				data_can_mit_send(&motor_chassis[i*2]);
				delay_us(mit_delay_d[1]);
			}else//否则就采集数据  电机扭矩均为0
			{
				data_can_sample_only(&motor_chassis[i]);
				delay_us(mit_delay_d[0]);
				data_can_sample_only(&motor_chassis[i*2]);
				delay_us(mit_delay_d[1]);
			}
		}
	#endif
	
	switch(en_mit_out)//----------------使能信号的状态机
	{
		case 0://关闭状态
		auto_off_t+=dt;
		if(auto_off_t>0.5&&1)//自动关闭周期发送
			{
				auto_off_t=0;
				for(i=0;i<14;i++){
					mit_motor_mode_en(i,0);
					delay_us(mit_delay);
				}
			}else if(en_test||en_all)
			{
				for(i=0;i<14;i++){
					mit_motor_mode_en(i,1);
					delay_us(mit_delay);
				}
				en_mit_out++;
				auto_off_t=0;
			}
		break;
		case 1:
			auto_off_t+=dt;
			if(auto_off_t>0.5&&1)//
			{
				auto_off_t=0;
				en_mit_out++;
			}
			for(i=0;i<14;i++){
				mit_motor_mode_en(i,1);
				delay_us(mit_delay);
			}				
		break;
		case 2://使能后
			if(!en_test&&!en_all)//触发式关闭
			{
				for(i=0;i<14;i++){
					mit_motor_mode_en(i,0);
					delay_us(mit_delay);
				}
				en_mit_out=0;
			}
		break;
	}
	
	for(i=0;i<4;i++)
		state_reg[i]=state_mit[i];	
}